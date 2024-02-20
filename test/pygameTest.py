import pygame
import sys
import numpy as np

# Initialize Pygame
pygame.init()

# Constants for screen dimensions
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (0, 0, 255)
GRAY = (200, 200, 200)

# Create the screen
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Pygame Dynamic Plotting")

# Define fonts
font = pygame.font.Font(None, 36)

# Define the data for plotting (you can replace this with your own data)
x = np.linspace(0, 10, 100)
y = np.sin(x)

# Create a function to draw a plot with title, gridlines, and axis labels
def draw_plot(surface, x_data, y_data, title, x_label, y_label):
    x_min, x_max = min(x_data), max(x_data)
    y_min, y_max = min(y_data), max(y_data)

    x_range = (x_min, x_max)
    y_range = (y_min, y_max)

    x_offset = 0.1 * surface.get_width()
    y_offset = 0.1 * surface.get_height()

    scaled_x = (x_data - x_range[0]) / (x_range[1] - x_range[0])
    scaled_y = 1 - (y_data - y_range[0]) / (y_range[1] - y_range[0])
    plot_points = [(int(x * (surface.get_width() - 2 * x_offset)) + x_offset,
                    int(y * (surface.get_height() - 2 * y_offset)) + y_offset) for x, y in zip(scaled_x, scaled_y)]

    # Draw gridlines
    for i in range(1, 11):
        x_pos = x_offset + i * (surface.get_width() - 2 * x_offset) / 10
        y_pos = y_offset + i * (surface.get_height() - 2 * y_offset) / 10
        pygame.draw.line(surface, GRAY, (x_pos, y_offset), (x_pos, surface.get_height() - y_offset), 1)
        pygame.draw.line(surface, GRAY, (x_offset, y_pos), (surface.get_width() - x_offset, y_pos), 1)

    # Draw the plot
    pygame.draw.lines(surface, BLUE, False, plot_points, 2)

    # Draw title
    title_text = font.render(title, True, BLACK)
    title_rect = title_text.get_rect()
    title_rect.centerx = surface.get_width() / 2
    title_rect.top = y_offset - 20
    surface.blit(title_text, title_rect)

    # Draw x-axis label
    x_label_text = font.render(x_label, True, BLACK)
    x_label_rect = x_label_text.get_rect()
    x_label_rect.centerx = surface.get_width() / 2
    x_label_rect.top = surface.get_height() - y_offset + 10
    surface.blit(x_label_text, x_label_rect)

    # Draw y-axis label with rotation
    y_label_text = pygame.font.Font(None, 36).render(y_label, True, BLACK)
    y_label_text = pygame.transform.rotate(y_label_text, 90)
    y_label_rect = y_label_text.get_rect()
    y_label_rect.centerx = x_offset - 40
    y_label_rect.centery = surface.get_height() / 2
    surface.blit(y_label_text, y_label_rect)

# Game loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Clear the screen
    screen.fill(WHITE)

    # Draw the plot in a specified box with title and labels
    plot_box_rect = pygame.Rect(100, 100, 600, 400)
    pygame.draw.rect(screen, BLACK, plot_box_rect, 2)
    draw_plot(screen, x, y, "Sine Wave Plot", "Time (s)", "Amplitude")

    # Update the display
    pygame.display.flip()

# Quit Pygame
pygame.quit()
