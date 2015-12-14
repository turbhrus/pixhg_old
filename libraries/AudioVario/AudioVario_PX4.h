/*
  ToneAlarm PX4 driver
*/
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __AUDIO_VARIO_PX4_H__
#define __AUDIO_VARIO_PX4_H__

class AudioVario_PX4
{
public:
    // constructor
    AudioVario_PX4() : _audiovario_fd(-1) {}

    /// init - initialised the tone alarm
    bool init(void);

    // start a new beep and return the start time in ms
    // pos is true if vario > 0, false otherwise
    uint32_t beep(uint8_t note, uint16_t duration, bool pos);


private:

    // note to name as a string: "C","C#",...,"B"
    void tone_name(uint8_t note, char* str);

    // note to octave number as a string: "0" to "6"
    uint8_t octave(uint8_t note);


    // convert beep duration into equivalent tempo
    uint8_t tempo(uint16_t duration);


    /// play_tune - play one of the pre-defined tunes
    void play_tone(const uint8_t tone_index);

    // play_string - play tone specified by the provided string of notes
    void play_string(const char *str);

    // stop_cont_tone - stop playing the currently playing continuous tone
    void stop_cont_tone();

    // check_cont_tone - check if we should begin playing a continuous tone
    void check_cont_tone();

    int _audiovario_fd;      // file descriptor for the vario

    int8_t _cont_tone_playing;
    int8_t _tone_playing;
    uint32_t _tone_beginning_ms;  // TODO remove me

    struct Tone {
        const char *str;
        const uint8_t continuous : 1;
    };

    const static char tone_names[12][3];
};

#endif // __AUDIO_VARIO_PX4_H__
