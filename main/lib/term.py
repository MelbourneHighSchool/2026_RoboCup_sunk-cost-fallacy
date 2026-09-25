"""Manage the terminal output"""
import threading
import curses
import textwrap
import time
import math

def radio_select(title:str, options:list[str], descriptions:list[str]=[], preset:int=0, tip:str=None) -> int | bool:
    """Return the index of the selected option, or False if the user quits"""
    def menu(stdscr):
        key = None
        cur = 0
        if preset:
            if preset < 0 or preset >= len(options):
                raise ValueError("Preset index out of range")
            cur = preset
        if descriptions and len(descriptions) != len(options):
            raise ValueError("Descriptions length must match options length")
        while 1:
            stdscr.erase()
            height, width = stdscr.getmaxyx()

            # if key == ord("q"):
            #     break
            if key == curses.KEY_UP:
                cur = (cur - 1) % len(options)
            elif key == curses.KEY_DOWN:
                cur = (cur + 1) % len(options)
            elif key in [ord(" "), curses.KEY_ENTER, 10, 13]:
                return cur

            # Create new window
            frame_width = max(len(a) + 2 for a in [*options] + [title]) + 2
            frame_height = len(options) + 2

            frame_win = curses.newwin(frame_height + 2, frame_width + 2, height // 2 - len(options) // 2 - 1, width // 2 - frame_width // 2)
            frame_win.box()

            # Title
            frame_win.addstr(0, 2, f" {title} ")
            # Options
            for i, option in enumerate(options):
                if i == cur:
                    frame_win.attron(curses.A_REVERSE)
                    frame_win.addstr(i + 2, 2, option)
                    frame_win.attroff(curses.A_REVERSE)
                else:
                    frame_win.addstr(i + 2, 2, option)

            # Tip
            if tip:
                stdscr.addstr(height - 2, width - 1 - len(tip), tip)

            # Description
            if descriptions and 0 <= cur < len(descriptions):
                desc = textwrap.wrap(descriptions[cur], width - 6)
                for i, line in enumerate(desc):
                    stdscr.addstr(height - 1 - len(desc) + i, 2, line)
                    
            # Quit
            # stdscr.addstr(height - 2, width - 18, "Press 'q' to quit")

            # Refresh
            stdscr.refresh()
            frame_win.refresh()

            curses.curs_set(0)

            key = stdscr.getch()

        return False

    return curses.wrapper(menu)

def multi_select(title:str, options:list[str], descriptions:list[str]=[], preset:list[bool]=None, tip:str=None) -> list[bool] | bool:
    """Return a list of booleans indicating which options were selected, or False if the user quits"""
    def menu(stdscr):
        key = None
        cur = 0
        selected = [False] * len(options)
        if preset:
            if len(preset) != len(options):
                raise ValueError("Preset length must match options length")
            selected = preset
        while 1:
            stdscr.erase()
            height, width = stdscr.getmaxyx()

            # if key == ord("q"):
            #     break
            if key == curses.KEY_UP:
                cur = (cur - 1) % (len(options) + 1)
            elif key == curses.KEY_DOWN:
                cur = (cur + 1) % (len(options) + 1)
            elif key in [ord(" "), curses.KEY_ENTER, 10, 13]:
                if cur == len(options):
                    return selected
                selected[cur] = not selected[cur]

            # Create new window
            frame_width = max(len(a) + 4 for a in [*options] + [title]) + 2
            frame_height = len(options) + 2

            frame_win = curses.newwin(frame_height + 2, frame_width + 2, height // 2 - len(options) // 2 - 1, width // 2 - frame_width // 2)
            frame_win.box()

            # Title
            frame_win.addstr(0, 3, f" {title} ")
            # Options
            for i, option in enumerate(options):
                prefix = "[x]" if selected[i] else "[ ]"
                if i == cur:
                    frame_win.attron(curses.A_REVERSE)
                    frame_win.addstr(i + 2, 2, f"{prefix} {option}")
                    frame_win.attroff(curses.A_REVERSE)
                else:
                    frame_win.addstr(i + 2, 2, f"{prefix} {option}")

            # Submit
            if cur == len(options):
                frame_win.attron(curses.A_REVERSE)
                frame_win.addstr(frame_height + 1, frame_width - 7, " [OK] ")
                frame_win.attroff(curses.A_REVERSE)
            else:
                frame_win.addstr(frame_height + 1, frame_width - 7, " [OK] ")
        
            # Tip
            if tip:
                stdscr.addstr(height - 2, 2, tip)

            # Description
            if descriptions and 0 <= cur < len(descriptions):
                desc = textwrap.wrap(descriptions[cur], width - 6)
                for i, line in enumerate(desc):
                    stdscr.addstr(height - 1 - len(desc) + i, 2, line)

            # Quit
            # stdscr.addstr(height - 2, width - 18, "Press 'q' to quit")

            # Refresh
            stdscr.refresh()
            frame_win.refresh()

            curses.curs_set(0)

            key = stdscr.getch()

        return False

    return curses.wrapper(menu)

def _clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))

class HSVInput:
    def __init__(self, title:str, low_preset:tuple[int, int, int]=(0, 0, 0), high_preset:tuple[int, int, int]=(0, 0, 0), tip:str=None):
        self.title = title
        self.tip = tip
        self.low = [_clamp(low_preset[0], 0, 255), _clamp(low_preset[1], 0, 255), _clamp(low_preset[2], 0, 255)]
        self.high = [_clamp(high_preset[0], 0, 255), _clamp(high_preset[1], 0, 255), _clamp(high_preset[2], 0, 255)]
    
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

        self.quit = False

    def is_running(self):
        return self.running

    def _run(self):
        def main(stdscr):
            cur = 0
            key = None
            while self.running:
                stdscr.erase()
                height, width = stdscr.getmaxyx()
                low, high = self.low[:], self.high[:]
                
                if key == ord("q"):
                    self.quit = True
                    break
                elif key in [ord(" "), curses.KEY_ENTER, 10, 13]:
                    break
                elif key == curses.KEY_UP:
                    cur = (cur - 1) % 6
                elif key == curses.KEY_DOWN:
                    cur = (cur + 1) % 6
                elif key == curses.KEY_LEFT:
                    if cur < 3:
                        low[cur] = (low[cur] - 1) % 256
                    else:
                        high[cur - 3] = (high[cur - 3] - 1) % 256
                elif key == curses.KEY_RIGHT:
                    if cur < 3:
                        low[cur] = (low[cur] + 1) % 256
                    else:
                        high[cur - 3] = (high[cur - 3] + 1) % 256
                
                    
    
                # Create new windows
                frame_width = 16
                frame_height = 7
    
                frame_low = curses.newwin(frame_height, frame_width, height // 2 - frame_height // 2, width // 2 - frame_width - 1)
                frame_high = curses.newwin(frame_height, frame_width, height // 2 - frame_height // 2, width // 2 + 1)

                frame_low.box()
                frame_high.box()
    
                # Title
                stdscr.addstr(1, width // 2 - len(self.title) // 2, self.title)
                frame_low.addstr(0, 2, f" Low ")
                frame_high.addstr(0, 2, f" High ")
                
                # Values
                for i in range(3):
                    if i == cur:
                        frame_low.attron(curses.A_REVERSE)
                    frame_low.addstr(i + 2, 2, f"{['H', 'S', 'V'][i]}: {low[i]:>3}")
                    if i == cur:
                        frame_low.attroff(curses.A_REVERSE)

                for i in range(3):
                    if i + 3 == cur:
                        frame_high.attron(curses.A_REVERSE)
                    frame_high.addstr(i + 2, 2, f"{['H', 'S', 'V'][i]}: {high[i]:>3}")
                    if i + 3 == cur:
                        frame_high.attroff(curses.A_REVERSE)
            
                # Tip
                if self.tip:
                    stdscr.addstr(height - 2, 2, self.tip)
    
                # Continue
                stdscr.addstr(height - 2, width - 26, "Press 'enter' to continue")

                # Quit
                stdscr.addstr(height - 3, width - 18, "Press 'q' to quit")
    
                # Refresh
                stdscr.refresh()
                frame_low.refresh()
                frame_high.refresh()
    
                curses.curs_set(0)

                # Save values
                self.low = low
                self.high = high
    
                key = stdscr.getch()

            self.running = False    
            return True

        return curses.wrapper(main)
    
class CenterInput:
    def __init__(self, title:str, low:int, high:int, default:list[int]=None, tip:str=None):
        self.title = title
        self.low = max(0, low)
        self.high = max(0, high)

        if not default or len(default) != 2:
            self.center = [self.low + (self.high - self.low) // 2, self.low + (self.high - self.low) // 2]
        else:
            self.center = [_clamp(default[0], self.low, self.high), _clamp(default[1], self.low, self.high)]

        self.tip = tip

        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

        self.quit = False

    def is_running(self):
        return self.running

    def _run(self):
        def main(stdscr):
            cur = 0
            key = None
            x, y = self.center[:]
            while self.running:
                stdscr.erase()
                height, width = stdscr.getmaxyx()
                
                if key == ord("q"):
                    self.quit = True
                    break
                elif key in [ord(" "), curses.KEY_ENTER, 10, 13]:
                    break
                elif key == curses.KEY_UP:
                    cur = (cur - 1) % 2
                elif key == curses.KEY_DOWN:
                    cur = (cur + 1) % 2
                elif key == curses.KEY_LEFT:
                    if cur == 0:
                        x = (x - 1) % (self.high + 1)
                    else:
                        y = (y - 1) % (self.high + 1)
                elif key == curses.KEY_RIGHT:
                    if cur == 0:
                        x = (x + 1) % (self.high + 1)
                    else:
                        y = (y + 1) % (self.high + 1)
    
                # Create new windows
                frame_width = 16
                frame_height = 7

                frame_main = curses.newwin(frame_height, frame_width, height // 2 - frame_height // 2, width // 2 - frame_width // 2)
                
                frame_main.addstr(0, 2, f"Centre")
                
                # Values
                for i in range(2):
                    if i == cur:
                        frame_main.attron(curses.A_REVERSE)
                    frame_main.addstr(i + 2, 2, f"{['X', 'Y'][i]}: {x if i == 0 else y:>3}")
                    if i == cur:
                        frame_main.attroff(curses.A_REVERSE)
            
                # Tip
                if self.tip:
                    stdscr.addstr(height - 2, 2, self.tip)
    
                # Continue
                stdscr.addstr(height - 2, width - 26, "Press 'enter' to continue")

                # Quit
                stdscr.addstr(height - 3, width - 18, "Press 'q' to quit")
    
                # Refresh
                stdscr.refresh()
                frame_main.refresh()
    
                curses.curs_set(0)

                # Save values 
                self.center = [x, y]
    
                key = stdscr.getch()

            self.running = False    
            return True

        return curses.wrapper(main)

        

if __name__ == "__main__":
    # options = ["Option 1", "Option 2", "Option 3"]
    # selected = radio_select("Select an option", options, "1/2")
    # selected_multi = multi_select("Select multiple options", options, "2/2")

    # print(f"Selected: {selected}\nSelected multiple: {selected_multi}")

    HSVInput("Test HSV Input", (0, 0, 0), (255, 255, 255), "Use arrow keys to adjust values.")
    while True:
        time.sleep(1)