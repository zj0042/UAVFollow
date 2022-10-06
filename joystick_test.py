import pygame

PYGAME_FPS = 30
pygame.init()
pygame.joystick.init()
joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
print(joysticks)
pygame.display.set_caption("Tello video stream")
screen = pygame.display.set_mode([int(960 * 0.9), int(720 * 0.9)])
pygame.time.set_timer(pygame.USEREVENT + 1, 1000 // PYGAME_FPS)

should_stop = False

while not should_stop:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            break
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_SPACE:
                should_stop = True

    screen.fill([0, 0, 0])
    pygame.display.update()
