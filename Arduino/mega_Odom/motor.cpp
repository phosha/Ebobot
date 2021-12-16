#pragma once
#include <Arduino.h>


class Motor {
  public:
    Motor(byte spd_pin, byte dir_pin, byte stp_pin) {
      _spd_pin = spd_pin;
      _dir_pin = dir_pin;
      _stp_pin = stp_pin;
    }

    void begin() {
      pinMode(_spd_pin, OUTPUT);
      pinMode(_dir_pin, OUTPUT);
      pinMode(_stp_pin, OUTPUT);
      digitalWrite(_spd_pin, LOW);
      digitalWrite(_dir_pin, HIGH);
      digitalWrite(_stp_pin, HIGH);
    }

    void run(bool dir, byte spd) {
      digitalWrite(_dir_pin, dir);
      digitalWrite(_stp_pin, !dir);
      analogWrite(_spd_pin, spd);
    }

    void go(int spd) {


      if (spd < 0) {
        spd = spd * (-1);
        _dir = 0;
        if (spd > 255)spd = 255;
        digitalWrite(_dir_pin, _dir);
        digitalWrite(_stp_pin, !_dir);
        analogWrite(_spd_pin, spd);
      }
      else {
        _dir = 1;
        if(spd > 255)spd = 255;
        digitalWrite(_dir_pin, _dir);
        digitalWrite(_stp_pin, !_dir);
        analogWrite(_spd_pin, spd);
      }
    }

    void stop() {
      digitalWrite(_dir_pin, 1);
      digitalWrite(_stp_pin, 1);
      // analogWrite(_spd_pin,0);
    }


  private:
    byte _spd_pin, _dir_pin, _stp_pin;
    byte _spd;
    bool _dir, _stp;
};
