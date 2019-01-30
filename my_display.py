import matplotlib.pyplot as plt


def display_points(coordinates):
    plt.scatter(coordinates.x, coordinates.y, s=2, c='r', marker='o')
    plt.xlim(-2.5, 5.8)
    plt.ylim(-3.5, 3.91)
    plt.show()


def display_line(coordinates, line):
    plt.scatter(coordinates.x, coordinates.y, s=2, c='r', marker='o')

    if line.leftend is not None:
        x = [line.leftend[0], line.righttend[0]]
        y = [line.leftend[1], line.righttend[1]]
    else:
        if line.b == 0:
            x = [-line.c / line.a, -line.c / line.a]
            y = [-10, 10]
        else:
            x = [-10, 10]
            y = [(-line.c + 10 * line.a) / line.b, (-line.c - 10 * line.a) / line.b]
    plt.plot(x, y, c='b')

    plt.xlim(-2.5, 5.8)
    plt.ylim(-3.5, 3.91)
    plt.show()


def display_lines(coordinates, lines):
    plt.scatter(coordinates.x, coordinates.y, s=2, c='r', marker='o')

    for line in lines:
        if line.leftend is not None:
            x = [line.leftend[0], line.rightend[0]]
            y = [line.leftend[1], line.rightend[1]]
        else:
            if line.b == 0:
                x = [-line.c / line.a, -line.c / line.a]
                y = [-10, 10]
            else:
                x = [-10, 10]
                y = [(-line.c + 10 * line.a) / line.b, (-line.c - 10 * line.a) / line.b]
        plt.plot(x, y, c='b')

    plt.xlim(-2.5, 5.8)
    plt.ylim(-3.5, 3.91)
    plt.show()
