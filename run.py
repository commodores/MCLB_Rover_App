from FlaskApp import create_app, mavlink_listener, socketio

app = create_app()

socketio.start_background_task(mavlink_listener)

socketio.run(app)





## Start the background task for listening to MAVLink messages
#socketio.start_background_task(mavlink_listener)

#if __name__ == "__main__":
 #   socketio.run(app, host="rover.local", port=5000)
   