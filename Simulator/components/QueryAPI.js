import BaseAPI, { methodType } from "./BaseAPI";

export default class QueryAPI extends BaseAPI {
  // Query the path from backend server
  static query(obstacles, robotX, robotY, robotDir, stepCompression, callback) {
    /* Construct the content of the request 
		obstacles: the array of obstacles
		robotX: the x coordinate of the robot
		robotY: the y coordinate of the robot
		robotDir: the direction of the robot
		stepCompression: whether to enable step compression
		retrying: whether the robot is retrying
*/
    const directionMap = {
      0: 'NORTH',
      2: 'EAST',
      4: 'SOUTH',
      6: 'WEST'
    };

    // Build obstacles string in the format: id;x;y;dir | id;x;y;dir | ...
    const obstaclesStr = obstacles.map(ob => `${ob.id};${ob.x};${ob.y};${directionMap[ob.d] || ob.d}`).join(' | ');

    // Build the full content string in the expected backend format
    const content = `STATUS,OBSTACLE_LIST,false,${robotX};${robotY};${directionMap[robotDir] || robotDir},false,${obstaclesStr},true,true`;

    // Send the request to the backend server using fetch directly (since BaseAPI expects JSON)
    const host = "http://127.0.0.1:8000";
    fetch(host + "/path", {
      method: 'POST',
      headers: {
        'Content-Type': 'text/plain',  // Change to plain text since it's CSV-like
      },
      body: content,
    })
      .then(response => {
        if (!response.ok) {
          throw new Error(`HTTP error! status: ${response.status}`);
        }
        return response.json();
      })
      .then(res => {
        if (callback) {
          callback({
            data: res.data,  // Unwrap the backend's data property
            error: null,
          });
        }
      })
      .catch(err => {
        console.log(err);
        if (callback) {
          callback({
            data: null,
            error: err,
          });
        }
      });
  }
}
