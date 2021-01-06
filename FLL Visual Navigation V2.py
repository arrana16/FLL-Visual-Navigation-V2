import pygame
import warnings
warnings.filterwarnings("ignore")

done=False

bg = pygame.image.load('/Users/abdur-rahmanrana/Documents/Coding Projects/FLL Digital Ruler/FLL-Replay-Mat.gif')
robot_image = pygame.image.load('/Users/abdur-rahmanrana/Documents/Robot Things/Robot Models/robot17 Odometry Render.png')

screen = pygame.display.set_mode((1124,629))

pos = (100,-500)

robotWaypoints = []


start=True

waypoints = []

clock=pygame.time.Clock()

def imageTurn(angle, pos):
    global rotated_image
    w, h = robot_image.get_size()
    box = [pygame.math.Vector2(p) for p in [(0, 0), (w, 0), (w, -h), (0, -h)]]
    box_rotate = [p.rotate(angle) for p in box]

    min_box = (min(box_rotate, key=lambda p: p[0])[0], min(box_rotate, key=lambda p: p[1])[1])
    max_box = (max(box_rotate, key=lambda p: p[0])[0], max(box_rotate, key=lambda p: p[1])[1])

    pivot = pygame.math.Vector2(w/2, -h/2)
    pivot_rotate = pivot.rotate(angle)
    pivot_move   = pivot_rotate - pivot
    origin = (int(pos[0] + min_box[0] - pivot_move[0]), int(pos[1] - max_box[1] + pivot_move[1]))
    rotated_image = pygame.transform.rotate(robot_image, angle)
    screen.blit(rotated_image, origin)



run=True
turnAngle=0
x=0
y=0
while run:
    pygame.time.delay(60)
    screen.fill((0,0,0))
    screen.blit(bg, (0,0))

    imageWidth, imageHeight = robot_image.get_size()

    keys = pygame.key.get_pressed()

    pos = (pygame.mouse.get_pos()[0]-imageWidth/2, pygame.mouse.get_pos()[1]-imageHeight/2)

    mousePos = (round(pygame.mouse.get_pos()[0]*1.81240063593), round(pygame.mouse.get_pos()[1]*-1.81240063593))

    imageTurn(turnAngle, pos)
    
    if keys[pygame.K_RIGHT]:
        turnAngle-=1

    if keys[pygame.K_LEFT]:
        turnAngle+=1

    if keys[pygame.K_w]:
        turnAngle=0

    if keys[pygame.K_d]:
        turnAngle=-90
    
    if keys[pygame.K_a]:
        turnAngle=90
    
    if keys[pygame.K_s]:
        turnAngle=180

    if keys[pygame.K_UP]:
        turnAngle-=5

    if keys [pygame.K_DOWN]:
        turnAngle+=5

    if keys [pygame.K_RETURN]:
        for _ in range(len(waypoints)):
            print(waypoints[_])

    if pygame.mouse.get_pressed()[0]:
        w, h = robot_image.get_size()
        box = [pygame.math.Vector2(p) for p in [(0, 0), (w, 0), (w, -h), (0, -h)]]
        box_rotate = [p.rotate(turnAngle) for p in box]

        min_box = (min(box_rotate, key=lambda p: p[0])[0], min(box_rotate, key=lambda p: p[1])[1])
        max_box = (max(box_rotate, key=lambda p: p[0])[0], max(box_rotate, key=lambda p: p[1])[1])

        pivot = pygame.math.Vector2(w/2, -h/2)
        pivot_rotate = pivot.rotate(turnAngle)
        pivot_move   = pivot_rotate - pivot
        origin = (pos[0] + min_box[0] - pivot_move[0], pos[1] - max_box[1] + pivot_move[1])
        robotWaypoints.append([pygame.Surface.copy(pygame.transform.rotate(robot_image, turnAngle)), origin])
        if start:
            waypoints.append(f"startPoint({mousePos}, {turnAngle*-1})")
        else:
            waypoints.append(f"waypoint({mousePos}, {turnAngle*-1})")
        start = False

    

    for x in range(len(robotWaypoints)):
        screen.blit(robotWaypoints[x][0], robotWaypoints[x][1])

    pygame.display.update()

    for event in pygame.event.get():
        if pygame.event.get()== pygame.QUIT:
            run=False
