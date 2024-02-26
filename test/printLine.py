import curses
import functools
import time

# Initialize data storage for function timings
func_timings = {}


def init_screen(stdscr):
    # Prevent typing from showing up
    curses.noecho()
    # React to keys instantly without requiring Enter
    curses.cbreak()
    # Enable special keys
    stdscr.keypad(True)


def restore_screen(stdscr):
    # Restore terminal to its original operating mode
    curses.nocbreak()
    stdscr.keypad(False)
    curses.echo()
    curses.endwin()


def log_time_cost(display_name=None):
    def decorator(func):
        nonlocal display_name
        if display_name is None:
            display_name = func.__name__

        @functools.wraps(func)
        def wrapper(stdscr=None, *args, **kwargs):
            nonlocal display_name
            start_time = time.time()
            result = func(*args, **kwargs)
            elapsed_time = time.time() - start_time

            if stdscr is not None:
                # Update function timings
                if display_name not in func_timings:
                    func_timings[display_name] = {
                        'last_exec_time': elapsed_time, 'exec_count': 1, 'avg_exec_time': elapsed_time}
                else:
                    func_data = func_timings[display_name]
                    func_data['last_exec_time'] = elapsed_time
                    func_data['exec_count'] += 1
                    func_data['avg_exec_time'] = (func_data['avg_exec_time'] * (
                        func_data['exec_count'] - 1) + elapsed_time) / func_data['exec_count']
                    func_timings[display_name] = func_data

                # Update the screen with the latest timings
                update_display(stdscr)

            return result
        return wrapper
    return decorator


def update_display(stdscr):
    stdscr.clear()  # Clear the screen to redraw the table
    row = 0
    stdscr.addstr(row, 0, "| Function Name | Last Exec Time | Avg Exec Time |")
    row += 1
    for func_name, data in func_timings.items():
        stdscr.addstr(
            row, 0, f"| {func_name} | {data['last_exec_time']:.4f} | {data['avg_exec_time']:.4f} |")
        row += 1
    stdscr.refresh()  # Refresh the screen to show the updates


def main(stdscr):
    # Example usage of the decorator
    @log_time_cost("Example Function")
    def example_function(seconds, stdscr=None):
        """Example function that sleeps for a given number of seconds."""
        time.sleep(seconds)

    # Simulate calling the decorated function, passing stdscr as an argument
    for _ in range(5):
        example_function(0.1, stdscr=stdscr)

    # Wait for a key press before exiting
    stdscr.getkey()


# To run the curses application
curses.wrapper(main)
