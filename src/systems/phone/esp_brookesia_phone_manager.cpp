/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <cmath>
#include "esp_brookesia_conf_internal.h"
#if !ESP_BROOKESIA_CONF_PHONE_MANAGER_ENABLE_DEBUG_LOG
#   define ESP_BROOKESIA_UTILS_DISABLE_DEBUG_LOG
#endif
#include "private/esp_brookesia_utils.h"
#include "esp_brookesia_phone_manager.hpp"
#include "esp_brookesia_phone.hpp"

using namespace std;
using namespace esp_brookesia::gui;

ESP_Brookesia_PhoneManager::ESP_Brookesia_PhoneManager(ESP_Brookesia_Core &core_in, ESP_Brookesia_PhoneHome &home_in,
        const ESP_Brookesia_PhoneManagerData_t &data_in):
    ESP_Brookesia_CoreManager(core_in, core_in.getCoreData().manager),
    home(home_in),
    data(data_in),
    _flags{},
    _home_active_screen(ESP_BROOKESIA_PHONE_MANAGER_SCREEN_MAX),
    _app_launcher_gesture_dir(ESP_BROOKESIA_GESTURE_DIR_NONE),
    _navigation_bar_gesture_dir(ESP_BROOKESIA_GESTURE_DIR_NONE),
    _gesture(nullptr),
    _recents_screen_drag_tan_threshold(0),
    _recents_screen_start_point{},
    _recents_screen_last_point{},
    _recents_screen_active_app(nullptr)
{
}

ESP_Brookesia_PhoneManager::~ESP_Brookesia_PhoneManager()
{
    ESP_BROOKESIA_LOGD("Destroy(0x%p)", this);
    if (!del()) {
        ESP_BROOKESIA_LOGE("Failed to delete");
    }
}

bool ESP_Brookesia_PhoneManager::calibrateData(const ESP_Brookesia_StyleSize_t screen_size, ESP_Brookesia_PhoneHome &home,
        ESP_Brookesia_PhoneManagerData_t &data)
{
    ESP_BROOKESIA_LOGD("Calibrate data");

    if (data.flags.enable_gesture) {
        ESP_BROOKESIA_CHECK_FALSE_RETURN(ESP_Brookesia_Gesture::calibrateData(screen_size, home, data.gesture), false,
                                         "Calibrate gesture data failed");
    }

    return true;
}

bool ESP_Brookesia_PhoneManager::begin(void)
{
    const ESP_Brookesia_RecentsScreen *recents_screen = home.getRecentsScreen();
    unique_ptr<ESP_Brookesia_Gesture> gesture = nullptr;
    lv_indev_t *touch = nullptr;

    ESP_BROOKESIA_LOGD("Begin(@0x%p)", this);
    ESP_BROOKESIA_CHECK_FALSE_RETURN(!checkInitialized(), false, "Already initialized");

    // Home
    lv_obj_add_event_cb(home.getMainScreen(), onHomeMainScreenLoadEventCallback, LV_EVENT_SCREEN_LOADED, this);

    // Gesture
    if (data.flags.enable_gesture) {
        // Get the touch device
        touch = _core.getTouchDevice();
        if (touch == nullptr) {
            ESP_BROOKESIA_LOGW("No touch device is set, try to use default touch device");

            touch = esp_brookesia_core_utils_get_input_dev(_core.getDisplayDevice(), LV_INDEV_TYPE_POINTER);
            ESP_BROOKESIA_CHECK_NULL_RETURN(touch, false, "No touch device is initialized");
            ESP_BROOKESIA_LOGW("Using default touch device(@0x%p)", touch);

            ESP_BROOKESIA_CHECK_FALSE_RETURN(_core.setTouchDevice(touch), false, "Core set touch device failed");
        }

        // Create and begin gesture
        gesture = make_unique<ESP_Brookesia_Gesture>(_core, data.gesture);
        ESP_BROOKESIA_CHECK_NULL_RETURN(gesture, false, "Create gesture failed");
        ESP_BROOKESIA_CHECK_FALSE_RETURN(gesture->begin(home.getSystemScreenObject()), false, "Gesture begin failed");
        ESP_BROOKESIA_CHECK_FALSE_RETURN(gesture->setMaskObjectVisible(false), false, "Hide mask object failed");
        ESP_BROOKESIA_CHECK_FALSE_RETURN(
            gesture->setIndicatorBarVisible(ESP_BROOKESIA_GESTURE_INDICATOR_BAR_TYPE_LEFT, false),
            false, "Set left indicator bar visible failed"
        );
        ESP_BROOKESIA_CHECK_FALSE_RETURN(
            gesture->setIndicatorBarVisible(ESP_BROOKESIA_GESTURE_INDICATOR_BAR_TYPE_RIGHT, false),
            false, "Set right indicator bar visible failed"
        );
        ESP_BROOKESIA_CHECK_FALSE_RETURN(
            gesture->setIndicatorBarVisible(ESP_BROOKESIA_GESTURE_INDICATOR_BAR_TYPE_BOTTOM, true),
            false, "Set bottom indicator bar visible failed"
        );

        _flags.enable_gesture_navigation = true;
        // Navigation events
        lv_obj_add_event_cb(gesture->getEventObj(), onGestureNavigationPressingEventCallback,
                            gesture->getPressingEventCode(), this);
        lv_obj_add_event_cb(gesture->getEventObj(), onGestureNavigationReleaseEventCallback,
                            gesture->getReleaseEventCode(), this);
        // Mask object and indicator bar events
        lv_obj_add_event_cb(gesture->getEventObj(), onGestureMaskIndicatorPressingEventCallback,
                            gesture->getPressingEventCode(), this);
        lv_obj_add_event_cb(gesture->getEventObj(), onGestureMaskIndicatorReleaseEventCallback,
                            gesture->getReleaseEventCode(), this);
    }

    if (gesture != nullptr) {
        // App Launcher
        lv_obj_add_event_cb(gesture->getEventObj(), onAppLauncherGestureEventCallback,
                            gesture->getPressingEventCode(), this);
        lv_obj_add_event_cb(gesture->getEventObj(), onAppLauncherGestureEventCallback,
                            gesture->getReleaseEventCode(), this);

        // Navigation Bar
        if (home.getNavigationBar() != nullptr) {
            lv_obj_add_event_cb(gesture->getEventObj(), onNavigationBarGestureEventCallback,
                                gesture->getPressingEventCode(), this);
            lv_obj_add_event_cb(gesture->getEventObj(), onNavigationBarGestureEventCallback,
                                gesture->getReleaseEventCode(), this);
        }
    }

    // Recents Screen
    if (recents_screen != nullptr) {
        // Hide recents_screen by default
        ESP_BROOKESIA_CHECK_FALSE_RETURN(recents_screen->setVisible(false), false, "Recents screen set visible failed");
        _recents_screen_drag_tan_threshold = tan(data.recents_screen.drag_snapshot_angle_threshold * M_PI / 180);
        lv_obj_add_event_cb(recents_screen->getEventObject(), onRecentsScreenSnapshotDeletedEventCallback,
                            recents_screen->getSnapshotDeletedEventCode(), this);
        // Register gesture event
        if (gesture != nullptr) {
            ESP_BROOKESIA_LOGD("Enable recents_screen gesture");
            lv_obj_add_event_cb(gesture->getEventObj(), onRecentsScreenGesturePressEventCallback,
                                gesture->getPressEventCode(), this);
            lv_obj_add_event_cb(gesture->getEventObj(), onRecentsScreenGesturePressingEventCallback,
                                gesture->getPressingEventCode(), this);
            lv_obj_add_event_cb(gesture->getEventObj(), onRecentsScreenGestureReleaseEventCallback,
                                gesture->getReleaseEventCode(), this);
        }
    }

    if (gesture != nullptr) {
        _gesture = std::move(gesture);
    }
    _flags.is_initialized = true;

    ESP_BROOKESIA_CHECK_FALSE_RETURN(processHomeScreenChange(ESP_BROOKESIA_PHONE_MANAGER_SCREEN_MAIN, nullptr), false,
                                     "Process screen change failed");

    return true;
}

bool ESP_Brookesia_PhoneManager::del(void)
{
    lv_obj_t *temp_obj = nullptr;

    ESP_BROOKESIA_LOGD("Delete phone manager(0x%p)", this);

    if (!checkInitialized()) {
        return true;
    }

    if (_gesture != nullptr) {
        _gesture.reset();
    }
    if (home.getRecentsScreen() != nullptr) {
        temp_obj = home.getRecentsScreen()->getEventObject();
        if (temp_obj != nullptr && checkLvObjIsValid(temp_obj)) {
            lv_obj_remove_event_cb(temp_obj, onRecentsScreenSnapshotDeletedEventCallback);
        }
    }
    _flags.is_initialized = false;
    _recents_screen_active_app = nullptr;

    return true;
}

bool ESP_Brookesia_PhoneManager::processAppRunExtra(ESP_Brookesia_CoreApp *app)
{
    ESP_Brookesia_PhoneApp *phone_app = static_cast<ESP_Brookesia_PhoneApp *>(app);

    ESP_BROOKESIA_CHECK_NULL_RETURN(phone_app, false, "Invalid phone app");
    ESP_BROOKESIA_LOGD("Process app(%p) run extra", phone_app);

    ESP_BROOKESIA_CHECK_FALSE_RETURN(processHomeScreenChange(ESP_BROOKESIA_PHONE_MANAGER_SCREEN_APP, phone_app), false,
                                     "Process screen change failed");

    return true;
}

bool ESP_Brookesia_PhoneManager::processAppResumeExtra(ESP_Brookesia_CoreApp *app)
{
    ESP_Brookesia_PhoneApp *phone_app = static_cast<ESP_Brookesia_PhoneApp *>(app);

    ESP_BROOKESIA_CHECK_NULL_RETURN(phone_app, false, "Invalid phone app");
    ESP_BROOKESIA_LOGD("Process app(%p) resume extra", phone_app);

    ESP_BROOKESIA_CHECK_FALSE_RETURN(processHomeScreenChange(ESP_BROOKESIA_PHONE_MANAGER_SCREEN_APP, phone_app), false,
                                     "Process screen change failed");

    return true;
}

bool ESP_Brookesia_PhoneManager::processAppCloseExtra(ESP_Brookesia_CoreApp *app)
{
    ESP_Brookesia_PhoneApp *phone_app = static_cast<ESP_Brookesia_PhoneApp *>(app);

    ESP_BROOKESIA_CHECK_NULL_RETURN(phone_app, false, "Invalid phone app");
    ESP_BROOKESIA_LOGD("Process app(%p) close extra", phone_app);

    if (getActiveApp() == app) {
        // Switch to the main screen to release the app resources
        ESP_BROOKESIA_CHECK_FALSE_RETURN(processHomeScreenChange(ESP_BROOKESIA_PHONE_MANAGER_SCREEN_MAIN, nullptr), false,
                                         "Process screen change failed");
        // If the recents_screen is visible, change back to the recents_screen
        if (home.getRecentsScreen()->checkVisible()) {
            ESP_BROOKESIA_CHECK_FALSE_RETURN(processHomeScreenChange(ESP_BROOKESIA_PHONE_MANAGER_SCREEN_RECENTS_SCREEN, nullptr), false,
                                             "Process screen change failed");
        }
    }

    return true;
}

bool ESP_Brookesia_PhoneManager::processHomeScreenChange(ESP_Brookesia_PhoneManagerScreen_t screen, void *param)
{
    ESP_BROOKESIA_LOGD("Process Screen Change(%d)", screen);
    ESP_BROOKESIA_CHECK_FALSE_RETURN(checkInitialized(), false, "Not initialized");
    ESP_BROOKESIA_CHECK_FALSE_RETURN(screen < ESP_BROOKESIA_PHONE_MANAGER_SCREEN_MAX, false, "Invalid screen");

    ESP_BROOKESIA_CHECK_FALSE_RETURN(processStatusBarScreenChange(screen, param), false, "Process status bar failed");
    ESP_BROOKESIA_CHECK_FALSE_RETURN(processNavigationBarScreenChange(screen, param), false, "Process navigation bar failed");
    ESP_BROOKESIA_CHECK_FALSE_RETURN(processGestureScreenChange(screen, param), false, "Process gesture failed");

    if (screen == ESP_BROOKESIA_PHONE_MANAGER_SCREEN_MAIN) {
        ESP_BROOKESIA_CHECK_FALSE_RETURN(home.processMainScreenLoad(), false, "Home load main screen failed");
    }
    _home_active_screen = screen;

    return true;
}

bool ESP_Brookesia_PhoneManager::processStatusBarScreenChange(ESP_Brookesia_PhoneManagerScreen_t screen, void *param)
{
    ESP_Brookesia_StatusBar *status_bar = home._status_bar.get();
    ESP_Brookesia_StatusBarVisualMode_t status_bar_visual_mode = ESP_BROOKESIA_STATUS_BAR_VISUAL_MODE_HIDE;
    const ESP_Brookesia_PhoneAppData_t *app_data = nullptr;

    ESP_BROOKESIA_LOGD("Process status bar when screen change");
    ESP_BROOKESIA_CHECK_FALSE_RETURN(checkInitialized(), false, "Not initialized");
    ESP_BROOKESIA_CHECK_FALSE_RETURN(screen < ESP_BROOKESIA_PHONE_MANAGER_SCREEN_MAX, false, "Invalid screen");

    if (status_bar == nullptr) {
        return true;
    }

    switch (screen) {
    case ESP_BROOKESIA_PHONE_MANAGER_SCREEN_MAIN:
        status_bar_visual_mode = home.getData().status_bar.visual_mode;
        break;
    case ESP_BROOKESIA_PHONE_MANAGER_SCREEN_APP:
        ESP_BROOKESIA_CHECK_NULL_RETURN(param, false, "Invalid param");
        app_data = &((ESP_Brookesia_PhoneApp *)param)->getActiveData();
        status_bar_visual_mode = app_data->status_bar_visual_mode;
        break;
    case ESP_BROOKESIA_PHONE_MANAGER_SCREEN_RECENTS_SCREEN:
        status_bar_visual_mode = home.getData().recents_screen.status_bar_visual_mode;
        break;
    default:
        ESP_BROOKESIA_CHECK_FALSE_RETURN(false, false, "Invalid screen");
        break;
    }
    ESP_BROOKESIA_LOGD("Visual Mode: status bar(%d)", status_bar_visual_mode);

    ESP_BROOKESIA_CHECK_FALSE_RETURN(status_bar->setVisualMode(status_bar_visual_mode), false,
                                     "Status bar set visual mode failed");

    return true;
}

bool ESP_Brookesia_PhoneManager::processNavigationBarScreenChange(ESP_Brookesia_PhoneManagerScreen_t screen, void *param)
{
    ESP_Brookesia_NavigationBar *navigation_bar = home._navigation_bar.get();
    ESP_Brookesia_NavigationBarVisualMode_t navigation_bar_visual_mode = ESP_BROOKESIA_NAVIGATION_BAR_VISUAL_MODE_HIDE;
    const ESP_Brookesia_PhoneAppData_t *app_data = nullptr;

    ESP_BROOKESIA_LOGD("Process navigation bar when screen change");
    ESP_BROOKESIA_CHECK_FALSE_RETURN(checkInitialized(), false, "Not initialized");
    ESP_BROOKESIA_CHECK_FALSE_RETURN(screen < ESP_BROOKESIA_PHONE_MANAGER_SCREEN_MAX, false, "Invalid screen");

    // Process status bar
    if (navigation_bar == nullptr) {
        return true;
    }

    switch (screen) {
    case ESP_BROOKESIA_PHONE_MANAGER_SCREEN_MAIN:
        navigation_bar_visual_mode = home.getData().navigation_bar.visual_mode;
        break;
    case ESP_BROOKESIA_PHONE_MANAGER_SCREEN_APP:
        ESP_BROOKESIA_CHECK_NULL_RETURN(param, false, "Invalid param");
        app_data = &((ESP_Brookesia_PhoneApp *)param)->getActiveData();
        navigation_bar_visual_mode = app_data->navigation_bar_visual_mode;
        break;
    case ESP_BROOKESIA_PHONE_MANAGER_SCREEN_RECENTS_SCREEN:
        navigation_bar_visual_mode = home.getData().recents_screen.navigation_bar_visual_mode;
        break;
    default:
        ESP_BROOKESIA_CHECK_FALSE_RETURN(false, false, "Invalid screen");
        break;
    }
    ESP_BROOKESIA_LOGD("Visual Mode: navigation bar(%d)", navigation_bar_visual_mode);

    _flags.enable_navigation_bar_gesture = (navigation_bar_visual_mode == ESP_BROOKESIA_NAVIGATION_BAR_VISUAL_MODE_SHOW_FLEX);
    ESP_BROOKESIA_CHECK_FALSE_RETURN(navigation_bar->setVisualMode(navigation_bar_visual_mode), false,
                                     "Navigation bar set visual mode failed");

    return true;
}

bool ESP_Brookesia_PhoneManager::processGestureScreenChange(ESP_Brookesia_PhoneManagerScreen_t screen, void *param)
{
    ESP_Brookesia_NavigationBar *navigation_bar = home._navigation_bar.get();
    ESP_Brookesia_NavigationBarVisualMode_t navigation_bar_visual_mode = ESP_BROOKESIA_NAVIGATION_BAR_VISUAL_MODE_HIDE;
    const ESP_Brookesia_PhoneAppData_t *app_data = nullptr;

    ESP_BROOKESIA_LOGD("Process gesture when screen change");
    ESP_BROOKESIA_CHECK_FALSE_RETURN(checkInitialized(), false, "Not initialized");
    ESP_BROOKESIA_CHECK_FALSE_RETURN(screen < ESP_BROOKESIA_PHONE_MANAGER_SCREEN_MAX, false, "Invalid screen");

    switch (screen) {
    case ESP_BROOKESIA_PHONE_MANAGER_SCREEN_MAIN:
        navigation_bar_visual_mode = home.getData().navigation_bar.visual_mode;
        _flags.enable_gesture_navigation = ((navigation_bar == nullptr) ||
                                            (navigation_bar_visual_mode == ESP_BROOKESIA_NAVIGATION_BAR_VISUAL_MODE_HIDE));
        _flags.enable_gesture_navigation_back = false;
        _flags.enable_gesture_navigation_home = false;
        _flags.enable_gesture_navigation_recents_app = _flags.enable_gesture_navigation;
        _flags.enable_gesture_show_mask_left_right_edge = false;
        _flags.enable_gesture_show_mask_bottom_edge = (_flags.enable_gesture_navigation ||
                (navigation_bar_visual_mode == ESP_BROOKESIA_NAVIGATION_BAR_VISUAL_MODE_SHOW_FLEX));
        _flags.enable_gesture_show_left_right_indicator_bar = false;
        _flags.enable_gesture_show_bottom_indicator_bar = _flags.enable_gesture_show_mask_bottom_edge;
        break;
    case ESP_BROOKESIA_PHONE_MANAGER_SCREEN_APP:
        ESP_BROOKESIA_CHECK_NULL_RETURN(param, false, "Invalid param");
        app_data = &((ESP_Brookesia_PhoneApp *)param)->getActiveData();
        navigation_bar_visual_mode = app_data->navigation_bar_visual_mode;
        _flags.enable_gesture_navigation = (app_data->flags.enable_navigation_gesture &&
                                            (navigation_bar_visual_mode != ESP_BROOKESIA_NAVIGATION_BAR_VISUAL_MODE_SHOW_FIXED));
        _flags.enable_gesture_navigation_back = (_flags.enable_gesture_navigation &&
                                                data.flags.enable_gesture_navigation_back);
        _flags.enable_gesture_navigation_home = (_flags.enable_gesture_navigation &&
                                                (navigation_bar_visual_mode == ESP_BROOKESIA_NAVIGATION_BAR_VISUAL_MODE_HIDE));
        _flags.enable_gesture_navigation_recents_app = _flags.enable_gesture_navigation_home;
        _flags.enable_gesture_show_mask_left_right_edge = (_flags.enable_gesture_navigation ||
                (navigation_bar_visual_mode == ESP_BROOKESIA_NAVIGATION_BAR_VISUAL_MODE_SHOW_FLEX));
        _flags.enable_gesture_show_mask_bottom_edge = (_flags.enable_gesture_navigation ||
                (navigation_bar_visual_mode == ESP_BROOKESIA_NAVIGATION_BAR_VISUAL_MODE_SHOW_FLEX));
        _flags.enable_gesture_show_left_right_indicator_bar = _flags.enable_gesture_show_mask_left_right_edge;
        _flags.enable_gesture_show_bottom_indicator_bar = _flags.enable_gesture_show_mask_bottom_edge;
        break;
    case ESP_BROOKESIA_PHONE_MANAGER_SCREEN_RECENTS_SCREEN:
        _flags.enable_gesture_navigation = false;
        _flags.enable_gesture_navigation_back = false;
        _flags.enable_gesture_navigation_home = false;
        _flags.enable_gesture_navigation_recents_app = false;
        _flags.enable_gesture_show_mask_left_right_edge = false;
        _flags.enable_gesture_show_mask_bottom_edge = false;
        _flags.enable_gesture_show_left_right_indicator_bar = false;
        _flags.enable_gesture_show_bottom_indicator_bar = false;
        break;
    default:
        ESP_BROOKESIA_CHECK_FALSE_RETURN(false, false, "Invalid screen");
        break;
    }
    ESP_BROOKESIA_LOGD("Gesture Navigation: all(%d), back(%d), home(%d), recents(%d)", _flags.enable_gesture_navigation,
                       _flags.enable_gesture_navigation_back, _flags.enable_gesture_navigation_home,
                       _flags.enable_gesture_navigation_recents_app);
    ESP_BROOKESIA_LOGD("Gesture Mask & Indicator: mask(left_right: %d, bottom: %d), indicator_left_right(%d), "
                       "indicator_bottom(%d)", _flags.enable_gesture_show_mask_left_right_edge,
                       _flags.enable_gesture_show_mask_bottom_edge, _flags.enable_gesture_show_left_right_indicator_bar,
                       _flags.enable_gesture_show_bottom_indicator_bar);

    if (!_flags.enable_gesture_show_left_right_indicator_bar) {
        ESP_BROOKESIA_CHECK_FALSE_RETURN(
            _gesture->setIndicatorBarVisible(ESP_BROOKESIA_GESTURE_INDICATOR_BAR_TYPE_LEFT, false), false,
            "Gesture set left indicator bar visible failed"
        );
        ESP_BROOKESIA_CHECK_FALSE_RETURN(
            _gesture->setIndicatorBarVisible(ESP_BROOKESIA_GESTURE_INDICATOR_BAR_TYPE_RIGHT, false), false,
            "Gesture set right indicator bar visible failed"
        );
    }
    ESP_BROOKESIA_CHECK_FALSE_RETURN(
        _gesture->setIndicatorBarVisible(
            ESP_BROOKESIA_GESTURE_INDICATOR_BAR_TYPE_BOTTOM, _flags.enable_gesture_show_bottom_indicator_bar
        ), false, "Gesture set bottom indicator bar visible failed"
    );

    return true;
}

void ESP_Brookesia_PhoneManager::onHomeMainScreenLoadEventCallback(lv_event_t *event)
{
    ESP_Brookesia_PhoneManager *manager = nullptr;
    ESP_Brookesia_RecentsScreen *recents_screen = nullptr;

    ESP_BROOKESIA_LOGD("Home main screen load event callback");
    ESP_BROOKESIA_CHECK_NULL_EXIT(event, "Invalid event");

    manager = static_cast<ESP_Brookesia_PhoneManager *>(lv_event_get_user_data(event));
    ESP_BROOKESIA_CHECK_NULL_EXIT(manager, "Invalid manager");
    recents_screen = manager->home.getRecentsScreen();

    // Only process the screen change if the recents_screen is not visible
    if ((recents_screen == nullptr) || !recents_screen->checkVisible()) {
        ESP_BROOKESIA_CHECK_FALSE_EXIT(
            manager->processStatusBarScreenChange(ESP_BROOKESIA_PHONE_MANAGER_SCREEN_MAIN, nullptr),
            "Process status bar failed");
        ESP_BROOKESIA_CHECK_FALSE_EXIT(
            manager->processNavigationBarScreenChange(ESP_BROOKESIA_PHONE_MANAGER_SCREEN_MAIN, nullptr),
            "Process navigation bar failed");
        ESP_BROOKESIA_CHECK_FALSE_EXIT(
            manager->processGestureScreenChange(ESP_BROOKESIA_PHONE_MANAGER_SCREEN_MAIN, nullptr),
            "Process gesture failed");
    }
}

void ESP_Brookesia_PhoneManager::onAppLauncherGestureEventCallback(lv_event_t *event)
{
    lv_event_code_t event_code = _LV_EVENT_LAST;
    ESP_Brookesia_PhoneManager *manager = nullptr;
    ESP_Brookesia_AppLauncher *app_launcher = nullptr;
    ESP_Brookesia_RecentsScreen *recents_screen = nullptr;
    ESP_Brookesia_Gesture *gesture = nullptr;
    ESP_Brookesia_GestureInfo_t *gesture_info = nullptr;
    ESP_Brookesia_GestureDirection_t dir_type = ESP_BROOKESIA_GESTURE_DIR_NONE;

    // ESP_BROOKESIA_LOGD("App launcher gesture event callback");
    ESP_BROOKESIA_CHECK_NULL_GOTO(event, end, "Invalid event");

    manager = static_cast<ESP_Brookesia_PhoneManager *>(lv_event_get_user_data(event));
    ESP_BROOKESIA_CHECK_NULL_GOTO(manager, end, "Invalid manager");
    gesture = manager->_gesture.get();
    ESP_BROOKESIA_CHECK_NULL_GOTO(gesture, end, "Invalid gesture");
    recents_screen = manager->home._recents_screen.get();
    app_launcher = &manager->home._app_launcher;
    ESP_BROOKESIA_CHECK_NULL_GOTO(app_launcher, end, "Invalid app launcher");
    event_code = lv_event_get_code(event);
    ESP_BROOKESIA_CHECK_FALSE_GOTO(
        (event_code == gesture->getPressingEventCode()) || (event_code == gesture->getReleaseEventCode()), end,
        "Invalid event code"
    );

    // Here is to prevent detecting gestures when the app exits, which could trigger unexpected behaviors
    if (event_code == gesture->getReleaseEventCode()) {
        if (manager->_flags.is_app_launcher_gesture_disabled) {
            manager->_flags.is_app_launcher_gesture_disabled = false;
            return;
        }
    }

    // Check if the app launcher and recents_screen are visible
    if (!app_launcher->checkVisible() || manager->_flags.is_app_launcher_gesture_disabled ||
            ((recents_screen != nullptr) && recents_screen->checkVisible())) {
        return;
    }

    dir_type = manager->_app_launcher_gesture_dir;
    // Check if the dir type is already set. If so, just ignore and return
    if (dir_type != ESP_BROOKESIA_GESTURE_DIR_NONE) {
        // Check if the gesture is released
        if (event_code == gesture->getReleaseEventCode()) {   // If so, reset the navigation type
            dir_type = ESP_BROOKESIA_GESTURE_DIR_NONE;
            goto end;
        }
        return;
    }

    gesture_info = (ESP_BROOKESIA_GESTUREArea truncated for brevity ...