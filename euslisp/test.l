#!/usr/bin/env roseus

;; --- 1. クラス定義 ---

(defclass hand-holding
  :slots ())

(defmethod hand-holding
  (:init ()
   (unless (ros::ok) (ros::roseus "hand_holding"))
   (ros::advertise "hand_angle" std_msgs::Float32MultiArray 1)
   (format t "[System] Hand Interface Initialized.~%")
  )

  (:publish-angle (angle-vec)
   (let ((pub-msg (instance std_msgs::Float32MultiArray :init)))
     (send pub-msg :data angle-vec)
     (ros::publish "hand_angle" pub-msg)))

  ;; --- 各モーション定義 ---

  ;; [1] five-finger-hook
  (:five-finger-hook ()
   (format t ">> Executing: Five Finger Hook~%")
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 0.1)
   (send self :publish-angle (float-vector 10.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector 10.0 -40.0 40.0 40.0 -40.0 35.0 1.5))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector 10.0 -35.0 35.0 35.0 -35.0 35.0 1.5))
   (ros::duration-sleep 2.0)
   (send self :publish-angle (float-vector 10.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 1.0)
   )

  ;; [2] interlocked-fingers
  (:interlocked-fingers ()
   (format t ">> Executing: Interlocked Fingers~%")
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 0.1)
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 -0.2))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector -20.0 -40.0 40.0 40.0 -40.0 35.0 -0.2))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector -20.0 -35.0 35.0 35.0 -35.0 35.0 -0.2))
   (ros::duration-sleep 2.0)
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 -0.2))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 0.5)
   )

  ;; [3] finger-hold
  (:finger-hold ()
   (format t ">> Executing: Finger Hold~%")
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 0.1)
   (send self :publish-angle (float-vector 10.0 60.0 -60.0 10.0 60.0 -60.0 0.2))
   (ros::duration-sleep 3.5)
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 1.0)
   )

  ;; [4] mutual-palm-grasp
  (:mutual-palm-grasp ()
   (format t ">> Executing: Mutual Palm Grasp~%")
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 0.1)
   (send self :publish-angle (float-vector 50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector 50.0 -40.0 40.0 40.0 -40.0 35.0 1.5))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector 50.0 -35.0 35.0 35.0 -35.0 35.0 1.5))
   (ros::duration-sleep 2.0)
   (send self :publish-angle (float-vector 50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 1.0)
   )

  ;; [5] finger-cradle-grasp
  (:finger-cradle-grasp ()
   (format t ">> Executing: Finger Cradle Grasp~%")
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 0.1)
   (send self :publish-angle (float-vector -50.0 -10.0 10.0 10.0 -10.0 -60.0 1.5))
   (ros::duration-sleep 4.0)
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 1.0)
   )

  ;; [6] four-finger-hook
  (:four-finger-hook ()
   (format t ">> Executing: Four Finger Hook~%")
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 0.1)
   (send self :publish-angle (float-vector 10.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector 10.0 -25.0 25.0 25.0 -25.0 25.0 1.5))
   (ros::duration-sleep 4.0)
   (send self :publish-angle (float-vector 10.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 1.0)
   )
  )


;; --- 2. メインループ ---

(defun main-loop ()
  (format t "
=== Hand Motion Controller ===
 [1] Five Finger Hook
 [2] Interlocked Fingers
 [3] Finger Hold
 [4] Mutual Palm Grasp
 [5] Finger Cradle Grasp
 [6] Four Finger Hook
 [q] Quit
------------------------------
")

  (let ((hand (instance hand-holding :init))
        (running t)
        (key nil))

    (while (and (ros::ok) running)
      (ros::spin-once)

      ;; キー入力チェック (select-streamで非同期監視)
      (when (select-stream (list *standard-input*) 0.001)
        (setq key (read-char *standard-input*))

        (case key
          (#\1 (send hand :five-finger-hook))
          (#\2 (send hand :interlocked-fingers))
          (#\3 (send hand :finger-hold))
          (#\4 (send hand :mutual-palm-grasp))
          (#\5 (send hand :finger-cradle-grasp))
          (#\6 (send hand :four-finger-hook))
          (#\q
           (format t "Exiting...~%")
           (setq running nil))
          ;; 改行などは無視
          (#\newline )
          (t (format t "Unknown key: ~C~%" key))
          )

        ;; 動作完了後にメニューを再表示（任意）
        (unless (member key '(#\newline))
           (format t "Ready. [1-6] to play motion, [q] to quit.~%"))
        )

      (ros::duration-sleep 0.1) ;; CPU負荷軽減
      )
    ))

;; --- 実行 ---
(main-loop)
