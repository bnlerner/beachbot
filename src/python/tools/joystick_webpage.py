from flask import Flask, render_template, request, jsonify

app = Flask(__name__)

@app.route('/')
def index() -> None:
    return render_template('index.html')

@app.route('/joystick_input', methods=['POST'])
def joystick_input():
    # Capture joystick input from POST request
    data = request.json
    x = data.get('x')
    y = data.get('y')
    # NOTE: Order not guaranteed so ensure that we stop for ~0.5s before moving again.
    stopped = data.get('stopped')
    print(f"Joystick Position - X: {x}, Y: {y}, Stopped: {stopped}")
    return jsonify({'status': 'success', 'message': 'Data received'})

if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)
