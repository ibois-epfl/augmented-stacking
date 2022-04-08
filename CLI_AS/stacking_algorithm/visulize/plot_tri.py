
def plot_triangle_cgal(filename):

    # read data by line
    def read_data(file_name):
        data = []
        with open(file_name, 'r') as f:
            for line in f:
                data.append(line.strip().split(' '))
        return data

    data = read_data(filename)

    # filter the data that has the length 3
    nodes = [i for i in data if len(i) == 3]
    cells = [i for i in data if len(i) == 4]
    infos = [i for i in data if len(i) == 1]

    # convert a list of string to float

    def convert_to_float(data):
        for i in range(len(data)):
            data[i] = [float(j) for j in data[i]]
        return data
    # conver a list of string to int

    def convert_to_int(data):
        for i in range(len(data)):
            data[i] = [int(j) for j in data[i] if j != '']
        return data

    nodes = convert_to_float(nodes)
    cells = convert_to_int(cells)
    infos = convert_to_int(infos)

    # connvert connectivity to line
    nb_cell = infos[2][0]

    def convert_cell_to_line(cells):
        lines = []
        for i in range(nb_cell):
            # every node in a cell is connected to other Nodes
            for start in range(4):
                for end in range(start, 4):
                    if cells[i][start] != 0 and cells[i][end] != 0:
                        lines.append([cells[i][start], cells[i][end]])
        return lines

    def convert_node_to_dict(nodes):
        n_dict = dict()
        for i in range(len(nodes)):
            n_dict[i+1] = nodes[i]
        return n_dict

    nodes = convert_node_to_dict(nodes)
    lines = convert_cell_to_line(cells)

    import plotly.express as px
    # plot multiply lines in 3d

    def plot_lines(lines, nodes):
        x = []
        y = []
        z = []
        for i in range(len(lines)):
            x.append(nodes[lines[i][0]][0])
            x.append(nodes[lines[i][1]][0])
            y.append(nodes[lines[i][0]][1])
            y.append(nodes[lines[i][1]][1])
            z.append(nodes[lines[i][0]][2])
            z.append(nodes[lines[i][1]][2])
        trace = px.line_3d(
            x=x,
            y=y,
            z=z
        )
        return trace

    trace = plot_lines(lines, nodes)
    return trace
