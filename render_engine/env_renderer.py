# Local, Self made classes
from logger.logger_config import setup_logger

# Python Libs
import math

# Installed Libs
import pygame

# Setup Logging
logger = setup_logger(__name__)

def scale_image(img, factor):
    size = round(img.get_width() * factor), round(img.get_height() * factor)
    return pygame.transform.scale(img, size)


def blit_rotate_center(win, image, top_left, angle):
    angle = -(angle*180)/math.pi-90
    player_image = pygame.transform.rotate(image, angle)
    player_rect = player_image.get_rect()
    player_rect.center = image.get_rect(topleft=top_left).center
    win.blit(player_image, player_rect)


class Resources:
    GRASS = scale_image(pygame.image.load(
        "render_engine/resources/grass.jpg"), 2.5)
    TRACK = scale_image(pygame.image.load(
        "render_engine/resources/track.png"), 0.9)

    TRACK_BORDER = scale_image(pygame.image.load(
        "render_engine/resources/track-border.png"), 0.9)
    TRACK_BORDER_MASK = pygame.mask.from_surface(TRACK_BORDER)

    FINISH = pygame.image.load("render_engine/resources/finish.png")
    FINISH_MASK = pygame.mask.from_surface(FINISH)

    RED_CAR = scale_image(pygame.image.load(
        "render_engine/resources/red-car.png"), 0.55)
    GREEN_CAR = scale_image(pygame.image.load(
        "render_engine/resources/green-car.png"), 0.55)


class Settings:
    # Window settings
    WIDTH, HEIGHT = Resources.TRACK.get_width(), Resources.TRACK.get_height()

    INFO_BOX_HEIGHT = 40
    HEIGHT += INFO_BOX_HEIGHT

    # [px] centre of car img
    CAR_X_OFFSET = Resources.RED_CAR.get_width()/2
    # [px] between front axle
    CAR_Y_OFFSET = Resources.RED_CAR.get_height()*0.25
    WB = Resources.RED_CAR.get_height()*0.65

class RenderEnvironment:
    def __init__(self, control_type, images, car_img, teach_mode, render_true_path, true_path=None):
        pygame.init()
        self.win = pygame.display.set_mode((Settings.WIDTH, Settings.HEIGHT))
        pygame.display.set_caption(
            f"Self driving with the {control_type} controller")
        self.car_img = car_img
        self.images = images
        self.TEACH_MODE = teach_mode
        self.RENDER_TRUE_PATH = render_true_path
        self.true_path = true_path

    def setup_clock(self):
        return pygame.time.Clock()

    def quit_render(self):
        pygame.quit()
        if self.TEACH_MODE:
            return self.true_path

    def _render_resources(self):
        for img, pos in self.images:
            self.win.blit(img, pos)

    def _render_text(self, text_str):
        font = pygame.font.SysFont('cambria', 28)
        text = font.render(text_str, True, (0, 0, 255), (0, 0, 0))
        textRect = text.get_rect()
        textRect.center = (Settings.WIDTH / 2,
                           Settings.HEIGHT-Settings.INFO_BOX_HEIGHT/2)
        self.win.blit(text, textRect)

    def _teach_mode_update(self, events):
        for event in events:
            if event.type == pygame.MOUSEBUTTONDOWN:
                pos = pygame.mouse.get_pos()
                if pos not in self.true_path:
                    self.true_path.append(pos)
        self._render_true_path()

    def _render_true_path(self):
        for i in self.true_path:
            pygame.draw.circle(self.win, (255, 0, 0), i, 2)

    def _drive_mode_update(self, text_string, car_x, car_y, car_yaw):
        self.draw_car(car_x, car_y, car_yaw)

        if self.RENDER_TRUE_PATH:
            self._render_true_path()
        self._render_text(text_string)

    def update_display(self, text_string="", car_x=0, car_y=0, car_yaw=0):
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.QUIT:
                logger.info("Got kill event")
                return False

        self._render_resources()

        if self.TEACH_MODE:
            self._teach_mode_update(events)
        else:
            self._drive_mode_update(text_string, car_x, car_y, car_yaw)

        pygame.display.update()
        return True

    def draw_car(self, car_x, car_y, car_yaw):
        """
        Red point represents front axle centre
        Blue point represents rear axle centre
        Renders car
        """
        rear_x = int(car_x - ((Settings.WB) * math.cos(car_yaw)))
        rear_y = int(car_y - ((Settings.WB) * math.sin(car_yaw)))
        angle = -(car_yaw*180)/math.pi-90
        player_image = pygame.transform.rotate(self.car_img, angle)
        player_rect = player_image.get_rect()
        player_rect.center = ((car_x+rear_x)//2,(car_y+rear_y)//2)
        self.win.blit(player_image, player_rect)
        pygame.draw.circle(self.win, (0, 0, 255), (rear_x,rear_y), 2)
        pygame.draw.circle(self.win, (255, 0, 0), (car_x, car_y), 2)

