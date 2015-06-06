#pragma once
typedef struct CTwBar TwBar; // structure CTwBar is not exposed.

class TwSettings{
public:
    TwBar* anttweakbar(){ return bar; }
    void tw_init(int width, int height);
    void tw_cleanup();
    void tw_resz(int width, int height);
    void tw_draw();
    void tw_add_ro(float &var, const char *name, const char *def="");
    void tw_add_ro(bool &var, const char *name, const char *def="");
    void tw_add(float &var, const char *name, const char *def="");
    void tw_add(bool &var, const char *name, const char *def="");
    void tw_add(int &var, const char *name, const char *def="");
private:
    TwBar* bar = 0 /*NULL*/;
};

/// --- Externally pre-instantiated settings
extern TwSettings* tw_settings;

