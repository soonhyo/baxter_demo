#!/usr/bin/env roseus
;; load the dependencies
(require :room73b2-askul-1200x700-desk-0 "models/room73b2-askul-1200x700-desk-0-object.l")
(load "package://baxtereus/baxter-interface.l")

;; add ros msgs
(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "geometry_msgs")

;; init baxter interface
(baxter-init)
;; reset pose for baxter
(send *baxter* :reset-pose)
;; send joint angles to real robot, to reset the real pose
(send *ri* :angle-vector (send *baxter* :angle-vector) 10000)

;; init table graphical model
(setq *table* (room73b2-askul-1200x700-desk-0))
(setq *table-pos* (float-vector 750 0 -900))
(send *table* :translate *table-pos*)

;; init ros node
(ros::roseus "baxter-demo")

(defun pick-from-bbox (bbox)
  (let (target-coords target-cube approach-coords base->camera (stamp (ros::time 0)))
    (if bbox
        (progn
          (send *tfl* :wait-for-transform
                "base" (send bbox :header :frame_id) stamp 10.0)
          ;; get transformation from base to camera
          (setq base->camera
                (send *tfl* :lookup-transform
                      "base" (send bbox :header :frame_id) stamp))
          (setq target-coords
                (send (send base->camera :copy-worldcoords)
                      :transform (ros::tf-pose->coords (send bbox :pose))))

          ;; change the target frame to align hand's frame
          (send target-coords :rotate (/ pi 2) :x)
          (send target-coords :rotate (/ pi 2) :z)

          ;; for visualization of target bbox (use mm in euslisp)
          (setq target-cube (make-cube (* 1000 (send bbox :dimensions :x))
                                       (* 1000 (send bbox :dimensions :y))
                                       (* 1000 (send bbox :dimensions :z))))
          (send target-cube :newcoords (send target-coords :copy-worldcoords))

          ;; approach robot arm to target coords
          (setq approach-coords (send target-coords :copy-worldcoords))
          (send approach-coords :translate #f(0 0 200) :world)

          ;; calculate inverse kinematics for right arm (virtual robot model)
          (send *baxter* :rarm :inverse-kinematics approach-coords)

          ;; for visualization in euslisp interface
          (objects (list *baxter* *table* target-coords target-cube approach-coords))
          (send *irtviewer* :draw-objects)

          ;; send the virtual robot model's jointstates to the real robot (5000ms)
          (send *ri* :angle-vector (send *baxter* :angle-vector) 5000)
          (send *ri* :wait-interpolation) ;; wait until finishing the movement

          ;; move right end effector position (virtual robot model)
          (send *baxter* :rarm :move-end-pos #f(0 0 -200) :world)
          ;; move the real robot
          (send *ri* :angle-vector (send *baxter* :angle-vector) 5000)
          (send *ri* :wait-interpolation)

          ;; grip the right hand (no hand's graphical model in euslisp)
          (send *ri* :start-grasp :rarm)
          (unix::sleep 1) ;; there is no wait function for grasping in baxter interface, wait 1s

          ;; move upward (virtual robot model)
          (send *baxter* :rarm :move-end-pos #f(0 0 200) :world)
          ;; move real robot
          (send *ri* :angle-vector (send *baxter* :angle-vector) 5000)
          (send *ri* :wait-interpolation)

          ;; move leftward
          (send *baxter* :rarm :move-end-pos #f(0 -200 0) :world)
          ;; move real robot
          (send *ri* :angle-vector (send *baxter* :angle-vector) 5000)
          (send *ri* :wait-interpolation)

          ;; move downward
          (send *baxter* :rarm :move-end-pos #f(0 0 -200) :world)
          ;; move real robot
          (send *ri* :angle-vector (send *baxter* :angle-vector) 5000)
          (send *ri* :wait-interpolation)

          ;; realease grasp
          (send *ri* :stop-grasp :rarm)
          (unix::sleep 1) ;; there is no wait function for grasping in baxter interface, wait 1s

          ;; move upward
          (send *baxter* :rarm :move-end-pos #f(0 0 200) :world)
          ;; move real robot
          (send *ri* :angle-vector (send *baxter* :angle-vector) 5000)
          (send *ri* :wait-interpolation)

          ;; reset pose
          (send *baxter* :reset-pose)
          (send *ri* :angle-vector (send *baxter* :angle-vector) 10000)
          )
        )
    )
  )

;; bbox-msg handler function
(defun bbox-msg-handler (bbox-msg)
  (let*
      ((boxes (send bbox-msg :boxes))
       (bbox (if boxes (car boxes))))
    (if bbox
        (pick-from-bbox bbox))))

;; subscribe bounding box once and save the msg to valuable
(setq bbox-msg (one-shot-subscribe "/segmentation_decomposer/boxes"
                                   jsk_recognition_msgs::BoundingBoxArray
                                   :timeout (* 10 1000)))

(bbox-msg-handler bbox-msg)
