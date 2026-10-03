struct Renderer {
	RGFW_window* window;
    bool display_quadtree = false;
    unsigned int pointsArrayObject, pointsBufferObject, quadTreeArrayObject, quadTreeBufferObject;
    unsigned int shaderProgram;
    int colorUniformLocation;
    std::vector<float> points;

	Renderer(RGFW_window* window_) {
		window = window_;
		RGFW_window_createContext_OpenGL(window, RGFW_getGlobalHints_OpenGL());
        gladLoadGL((GLADloadfunc)RGFW_getProcAddress_OpenGL);

        RGFW_window_makeCurrentContext_OpenGL(window);

        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

        glGenVertexArrays(1, &pointsArrayObject);
        glGenBuffers(1, &pointsBufferObject);

        glBindVertexArray(pointsArrayObject);
        glBindBuffer(GL_ARRAY_BUFFER, pointsBufferObject);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2*sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        glGenVertexArrays(1, &quadTreeArrayObject);
        glGenBuffers(1, &quadTreeBufferObject);
        glBindVertexArray(quadTreeArrayObject);
        glBindBuffer(GL_ARRAY_BUFFER, quadTreeBufferObject);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        const GLchar* vertexShaderSource =
            R"(#version 330 core
    layout(location = 0) in vec2 aPos;
    void main() {
        gl_Position = vec4(2*aPos-vec2(1,1), 0.0, 1.0);
        gl_PointSize = 1.2;
    })";
        unsigned int vertexShader;
        vertexShader = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
        glCompileShader(vertexShader);

        const GLchar* fragmentShaderSource =
            R"(#version 330 core
    uniform vec4 inColor;
    out vec4 FragColor;
    void main() {
        FragColor = inColor;
    })";
        unsigned int fragmentShader;
        fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
        glCompileShader(fragmentShader);

        shaderProgram = glCreateProgram();
        glAttachShader(shaderProgram, vertexShader);
        glAttachShader(shaderProgram, fragmentShader);
        glLinkProgram(shaderProgram);
        colorUniformLocation = glGetUniformLocation(shaderProgram, "inColor");
    }

    void render(const Simulation& simulation) {
        points.resize(simulation.positions.size()*2);
        for (std::size_t i=0; i<simulation.positions.size(); ++i) {
            points[2*i] = simulation.positions[i].x;
            points[2*i+1] = simulation.positions[i].y;
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (display_quadtree) {
            std::vector<glm::vec2> lineVertices = quadtree_lines(simulation.quadtree);
            glBindVertexArray(quadTreeArrayObject);
            glBindBuffer(GL_ARRAY_BUFFER, quadTreeBufferObject);
            glBufferData(GL_ARRAY_BUFFER, lineVertices.size() * sizeof(glm::vec2), lineVertices.data(), GL_DYNAMIC_DRAW);
            glLineWidth(1.0f);
            glUseProgram(shaderProgram);
            glUniform4f(colorUniformLocation, 0.0f, 0.0f, 0.6f, 1.0f);
            glDrawArrays(GL_LINES, 0, lineVertices.size());
        }

        glBindVertexArray(pointsArrayObject);
        glBindBuffer(GL_ARRAY_BUFFER, pointsBufferObject);
        glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(float), points.data(), GL_DYNAMIC_DRAW);
        glEnable(GL_PROGRAM_POINT_SIZE);
        glUseProgram(shaderProgram);
        glUniform4f(colorUniformLocation, 1.0f, 1.0f, 1.0f, 1.0f);
        glDrawArrays(GL_POINTS, 0, points.size());

		RGFW_window_swapBuffers_OpenGL(window);
    }
};
