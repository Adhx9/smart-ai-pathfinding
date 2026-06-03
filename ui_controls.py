
import pygame

COLOR_TEXT = (240, 240, 240)
COLOR_UI_BG = (40, 40, 50)
COLOR_BUTTON = (70, 130, 180)
COLOR_BUTTON_HOVER = (100, 160, 210)

def draw_text(surface, text, pos, size=18, color=COLOR_TEXT, align="left", font_name='consolas'):
    font = pygame.font.SysFont(font_name, size)
    surf = font.render(text, True, color)
    if align == "center":
        rect = surf.get_rect(center=pos)
    elif align == "right":
        rect = surf.get_rect(topright=pos)
    else:
        rect = surf.get_rect(topleft=pos)
    surface.blit(surf, rect)
    return rect

def draw_panel(surface, rect, title):
    pygame.draw.rect(surface, COLOR_UI_BG, rect, border_radius=8)
    pygame.draw.rect(surface, COLOR_TEXT, rect, 2, border_radius=8)
    title_rect = pygame.Rect(rect.x + 10, rect.y - 10, 220, 25)
    pygame.draw.rect(surface, COLOR_UI_BG, title_rect, border_radius=5)
    pygame.draw.rect(surface, COLOR_TEXT, title_rect, 1, border_radius=5)
    draw_text(surface, title, (title_rect.centerx, title_rect.centery), 16, align="center")
    return rect

class Button:
    def __init__(self, x, y, width, height, text, action=None):
        self.rect = pygame.Rect(x, y, width, height)
        self.text = text
        self.action = action
        self.hovered = False

    def draw(self, surface):
        color = COLOR_BUTTON_HOVER if self.hovered else COLOR_BUTTON
        pygame.draw.rect(surface, color, self.rect, border_radius=6)
        pygame.draw.rect(surface, COLOR_TEXT, self.rect, 2, border_radius=6)
        draw_text(surface, self.text, self.rect.center, 16, align="center")

    def check_hover(self, pos):
        self.hovered = self.rect.collidepoint(pos)
        return self.hovered

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.hovered and self.action:
                return self.action()
        return None

class ToggleButton(Button):
    def __init__(self, x, y, width, height, options, get_index_fn, set_index_fn):
        super().__init__(x, y, width, height, "", None)
        self.options = options
        self.get_index = get_index_fn
        self.set_index = set_index_fn

    def draw(self, surface):
        color = COLOR_BUTTON_HOVER if self.hovered else COLOR_BUTTON
        pygame.draw.rect(surface, color, self.rect, border_radius=6)
        pygame.draw.rect(surface, COLOR_TEXT, self.rect, 2, border_radius=6)
        label = f"Algo: {self.options[self.get_index()]}"
        draw_text(surface, label, self.rect.center, 16, align="center")

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.hovered:
                i = self.get_index()
                i = (i + 1) % len(self.options)
                self.set_index(i)
                return "toggled"
        return None
