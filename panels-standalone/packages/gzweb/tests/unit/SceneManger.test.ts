import { SceneManager } from "../../src/SceneManager";

// Tests for default construction
describe("scene construction", () => {
  // Create the scene object
  let sceneMgr: SceneManager = new SceneManager();

  test("scene connection status is disconnected", () => {
    expect(sceneMgr.getConnectionStatus()).toBe("disconnected");
  });

  test("model list to be empty", () => {
    expect(sceneMgr.getModels().length).toBe(0);
  });

  test("manipulation commit dispatches set_pose through gzweb", () => {
    const requestService = jest.fn();
    const emit = jest.fn();

    (sceneMgr as any).scene = {
      emitter: { emit },
    };
    (sceneMgr as any).transport = {
      getWorld: () => "empty_world",
      getAvailableTopics: () => [
        {
          topic: "/world/empty_world/dynamic_pose/info",
          msg_type: "gz.msgs.Pose_V",
        },
      ],
      requestService,
    };

    (sceneMgr as any).handleManipulationCommit({
      name: "mug",
      position: { x: -1.6, y: 2.3, z: 1.02 },
      orientation: { x: 0, y: 0, z: 0, w: 1 },
    });

    expect(requestService).toHaveBeenCalledWith(
      "/world/empty_world/set_pose",
      "gz.msgs.Pose",
      {
        name: "mug",
        position: { x: -1.6, y: 2.3, z: 1.02 },
        orientation: { x: 0, y: 0, z: 0, w: 1 },
      },
    );
    expect(emit).toHaveBeenCalledWith(
      "manipulation_dispatch",
      expect.objectContaining({
        ok: true,
        name: "mug",
        world: "empty_world",
        serviceName: "/world/empty_world/set_pose",
        msgType: "gz.msgs.Pose",
      }),
    );
  });
});
