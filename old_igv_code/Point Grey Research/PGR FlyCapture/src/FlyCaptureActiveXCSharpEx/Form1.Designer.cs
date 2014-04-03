namespace ActiveFlyCapCSharpEx
{
   partial class ActiveFlyCapForm
   {
      /// <summary>
      /// Required designer variable.
      /// </summary>
      private System.ComponentModel.IContainer components = null;

      /// <summary>
      /// Clean up any resources being used.
      /// </summary>
      /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
      protected override void Dispose(bool disposing)
      {
	 if (disposing && (components != null))
	 {
	    components.Dispose();
	 }
	 base.Dispose(disposing);
      }

      #region Windows Form Designer generated code

      /// <summary>
      /// Required method for Designer support - do not modify
      /// the contents of this method with the code editor.
      /// </summary>
      private void InitializeComponent()
      {
	 System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(ActiveFlyCapForm));
	 this.axActiveFlyCapControl = new AxActiveFlyCapLib.AxActiveFlyCapControl();
	 this.buttonStartStop = new System.Windows.Forms.Button();
	 this.buttonCamControlDlg = new System.Windows.Forms.Button();
	 this.groupBox1 = new System.Windows.Forms.GroupBox();
	 this.label3 = new System.Windows.Forms.Label();
	 this.buttonSetRegister = new System.Windows.Forms.Button();
	 this.buttonGetRegister = new System.Windows.Forms.Button();
	 this.label2 = new System.Windows.Forms.Label();
	 this.label1 = new System.Windows.Forms.Label();
	 this.textBox2 = new System.Windows.Forms.TextBox();
	 this.textBox1 = new System.Windows.Forms.TextBox();
	 this.buttonQuit = new System.Windows.Forms.Button();
	 this.groupBox2 = new System.Windows.Forms.GroupBox();
	 this.labelBusSpeed = new System.Windows.Forms.Label();
	 this.label21 = new System.Windows.Forms.Label();
	 this.labelBusNode = new System.Windows.Forms.Label();
	 this.label23 = new System.Windows.Forms.Label();
	 this.labelSensor = new System.Windows.Forms.Label();
	 this.label15 = new System.Windows.Forms.Label();
	 this.labelDCAMVer = new System.Windows.Forms.Label();
	 this.label13 = new System.Windows.Forms.Label();
	 this.labelCameraModel = new System.Windows.Forms.Label();
	 this.label8 = new System.Windows.Forms.Label();
	 this.labelCameraType = new System.Windows.Forms.Label();
	 this.label6 = new System.Windows.Forms.Label();
	 this.labelSerialNum = new System.Windows.Forms.Label();
	 this.label4 = new System.Windows.Forms.Label();
	 this.menuStrip1 = new System.Windows.Forms.MenuStrip();
	 this.fileToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
	 this.newCameraToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
	 this.toolStripSeparator1 = new System.Windows.Forms.ToolStripSeparator();
	 this.exitToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
	 this.cameraToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
	 this.saveImageToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
	 this.toolStripSeparator2 = new System.Windows.Forms.ToolStripSeparator();
	 this.cameraControlDialogToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
	 this.statusStrip1 = new System.Windows.Forms.StatusStrip();
	 this.toolStripStatusLabel1 = new System.Windows.Forms.ToolStripStatusLabel();
	 this.checkBoxAutoResize = new System.Windows.Forms.CheckBox();
	 this.checkBoxDrawShapes = new System.Windows.Forms.CheckBox();
	 ((System.ComponentModel.ISupportInitialize)(this.axActiveFlyCapControl)).BeginInit();
	 this.groupBox1.SuspendLayout();
	 this.groupBox2.SuspendLayout();
	 this.menuStrip1.SuspendLayout();
	 this.statusStrip1.SuspendLayout();
	 this.SuspendLayout();
	 // 
	 // axActiveFlyCapControl
	 // 
	 this.axActiveFlyCapControl.Enabled = true;
	 this.axActiveFlyCapControl.Location = new System.Drawing.Point(12, 148);
	 this.axActiveFlyCapControl.Name = "axActiveFlyCapControl";
	 this.axActiveFlyCapControl.OcxState = ((System.Windows.Forms.AxHost.State)(resources.GetObject("axActiveFlyCapControl.OcxState")));
	 this.axActiveFlyCapControl.Size = new System.Drawing.Size(640, 480);
	 this.axActiveFlyCapControl.TabIndex = 0;
	 this.axActiveFlyCapControl.TabStop = false;
	 this.axActiveFlyCapControl.CameraArrival += new AxActiveFlyCapLib._IActiveFlyCapControlEvents_CameraArrivalEventHandler(this.axFlyCapActiveXControl_CameraArrival);
	 this.axActiveFlyCapControl.CameraRemoval += new AxActiveFlyCapLib._IActiveFlyCapControlEvents_CameraRemovalEventHandler(this.axFlyCapActiveXControl_CameraRemoval);
	 this.axActiveFlyCapControl.ImageGrabbed += new System.EventHandler(this.axFlyCapActiveXControl_ImageGrabbed);
	 this.axActiveFlyCapControl.BusReset += new System.EventHandler(this.axFlyCapActiveXControl_BusReset);
	 // 
	 // buttonStartStop
	 // 
	 this.buttonStartStop.Location = new System.Drawing.Point(12, 634);
	 this.buttonStartStop.Name = "buttonStartStop";
	 this.buttonStartStop.Size = new System.Drawing.Size(109, 44);
	 this.buttonStartStop.TabIndex = 0;
	 this.buttonStartStop.Text = "Start/Stop";
	 this.buttonStartStop.UseVisualStyleBackColor = true;
	 this.buttonStartStop.Click += new System.EventHandler(this.buttonStartStop_Click);
	 // 
	 // buttonCamControlDlg
	 // 
	 this.buttonCamControlDlg.Location = new System.Drawing.Point(127, 634);
	 this.buttonCamControlDlg.Name = "buttonCamControlDlg";
	 this.buttonCamControlDlg.Size = new System.Drawing.Size(109, 44);
	 this.buttonCamControlDlg.TabIndex = 1;
	 this.buttonCamControlDlg.Text = "Camera Control Dialog";
	 this.buttonCamControlDlg.UseVisualStyleBackColor = true;
	 this.buttonCamControlDlg.Click += new System.EventHandler(this.buttonCamControlDlg_Click);
	 // 
	 // groupBox1
	 // 
	 this.groupBox1.Controls.Add(this.label3);
	 this.groupBox1.Controls.Add(this.buttonSetRegister);
	 this.groupBox1.Controls.Add(this.buttonGetRegister);
	 this.groupBox1.Controls.Add(this.label2);
	 this.groupBox1.Controls.Add(this.label1);
	 this.groupBox1.Controls.Add(this.textBox2);
	 this.groupBox1.Controls.Add(this.textBox1);
	 this.groupBox1.Location = new System.Drawing.Point(318, 27);
	 this.groupBox1.Name = "groupBox1";
	 this.groupBox1.Size = new System.Drawing.Size(185, 115);
	 this.groupBox1.TabIndex = 3;
	 this.groupBox1.TabStop = false;
	 this.groupBox1.Text = "Registers";
	 // 
	 // label3
	 // 
	 this.label3.AutoSize = true;
	 this.label3.Location = new System.Drawing.Point(159, 37);
	 this.label3.Name = "label3";
	 this.label3.Size = new System.Drawing.Size(15, 13);
	 this.label3.TabIndex = 6;
	 this.label3.Text = "H";
	 // 
	 // buttonSetRegister
	 // 
	 this.buttonSetRegister.Location = new System.Drawing.Point(97, 77);
	 this.buttonSetRegister.Name = "buttonSetRegister";
	 this.buttonSetRegister.Size = new System.Drawing.Size(77, 23);
	 this.buttonSetRegister.TabIndex = 3;
	 this.buttonSetRegister.Text = "Set";
	 this.buttonSetRegister.UseVisualStyleBackColor = true;
	 this.buttonSetRegister.Click += new System.EventHandler(this.buttonSetRegister_Click);
	 // 
	 // buttonGetRegister
	 // 
	 this.buttonGetRegister.Location = new System.Drawing.Point(11, 77);
	 this.buttonGetRegister.Name = "buttonGetRegister";
	 this.buttonGetRegister.Size = new System.Drawing.Size(77, 23);
	 this.buttonGetRegister.TabIndex = 2;
	 this.buttonGetRegister.Text = "Get";
	 this.buttonGetRegister.UseVisualStyleBackColor = true;
	 this.buttonGetRegister.Click += new System.EventHandler(this.buttonGetRegister_Click);
	 // 
	 // label2
	 // 
	 this.label2.AutoSize = true;
	 this.label2.Location = new System.Drawing.Point(8, 51);
	 this.label2.Name = "label2";
	 this.label2.Size = new System.Drawing.Size(34, 13);
	 this.label2.TabIndex = 3;
	 this.label2.Text = "Value";
	 // 
	 // label1
	 // 
	 this.label1.AutoSize = true;
	 this.label1.Location = new System.Drawing.Point(8, 25);
	 this.label1.Name = "label1";
	 this.label1.Size = new System.Drawing.Size(46, 13);
	 this.label1.TabIndex = 2;
	 this.label1.Text = "Register";
	 // 
	 // textBox2
	 // 
	 this.textBox2.Font = new System.Drawing.Font("Consolas", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
	 this.textBox2.Location = new System.Drawing.Point(62, 51);
	 this.textBox2.MaxLength = 8;
	 this.textBox2.Name = "textBox2";
	 this.textBox2.Size = new System.Drawing.Size(91, 20);
	 this.textBox2.TabIndex = 1;
	 this.textBox2.Text = "0";
	 this.textBox2.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
	 // 
	 // textBox1
	 // 
	 this.textBox1.Font = new System.Drawing.Font("Consolas", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
	 this.textBox1.Location = new System.Drawing.Point(62, 25);
	 this.textBox1.MaxLength = 8;
	 this.textBox1.Name = "textBox1";
	 this.textBox1.Size = new System.Drawing.Size(91, 20);
	 this.textBox1.TabIndex = 0;
	 this.textBox1.Text = "0";
	 this.textBox1.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
	 // 
	 // buttonQuit
	 // 
	 this.buttonQuit.Location = new System.Drawing.Point(543, 634);
	 this.buttonQuit.Name = "buttonQuit";
	 this.buttonQuit.Size = new System.Drawing.Size(109, 44);
	 this.buttonQuit.TabIndex = 2;
	 this.buttonQuit.Text = "Quit";
	 this.buttonQuit.UseVisualStyleBackColor = true;
	 this.buttonQuit.Click += new System.EventHandler(this.buttonQuit_Click);
	 // 
	 // groupBox2
	 // 
	 this.groupBox2.Controls.Add(this.labelBusSpeed);
	 this.groupBox2.Controls.Add(this.label21);
	 this.groupBox2.Controls.Add(this.labelBusNode);
	 this.groupBox2.Controls.Add(this.label23);
	 this.groupBox2.Controls.Add(this.labelSensor);
	 this.groupBox2.Controls.Add(this.label15);
	 this.groupBox2.Controls.Add(this.labelDCAMVer);
	 this.groupBox2.Controls.Add(this.label13);
	 this.groupBox2.Controls.Add(this.labelCameraModel);
	 this.groupBox2.Controls.Add(this.label8);
	 this.groupBox2.Controls.Add(this.labelCameraType);
	 this.groupBox2.Controls.Add(this.label6);
	 this.groupBox2.Controls.Add(this.labelSerialNum);
	 this.groupBox2.Controls.Add(this.label4);
	 this.groupBox2.Location = new System.Drawing.Point(12, 27);
	 this.groupBox2.Name = "groupBox2";
	 this.groupBox2.Size = new System.Drawing.Size(300, 115);
	 this.groupBox2.TabIndex = 5;
	 this.groupBox2.TabStop = false;
	 this.groupBox2.Text = "Camera Info";
	 // 
	 // labelBusSpeed
	 // 
	 this.labelBusSpeed.AutoSize = true;
	 this.labelBusSpeed.Location = new System.Drawing.Point(85, 94);
	 this.labelBusSpeed.Name = "labelBusSpeed";
	 this.labelBusSpeed.Size = new System.Drawing.Size(33, 13);
	 this.labelBusSpeed.TabIndex = 15;
	 this.labelBusSpeed.Text = "None";
	 // 
	 // label21
	 // 
	 this.label21.AutoSize = true;
	 this.label21.Location = new System.Drawing.Point(6, 94);
	 this.label21.Name = "label21";
	 this.label21.Size = new System.Drawing.Size(59, 13);
	 this.label21.TabIndex = 14;
	 this.label21.Text = "Bus Speed";
	 // 
	 // labelBusNode
	 // 
	 this.labelBusNode.AutoSize = true;
	 this.labelBusNode.Location = new System.Drawing.Point(85, 81);
	 this.labelBusNode.Name = "labelBusNode";
	 this.labelBusNode.Size = new System.Drawing.Size(33, 13);
	 this.labelBusNode.TabIndex = 13;
	 this.labelBusNode.Text = "None";
	 // 
	 // label23
	 // 
	 this.label23.AutoSize = true;
	 this.label23.Location = new System.Drawing.Point(6, 81);
	 this.label23.Name = "label23";
	 this.label23.Size = new System.Drawing.Size(62, 13);
	 this.label23.TabIndex = 12;
	 this.label23.Text = "Bus / Node";
	 // 
	 // labelSensor
	 // 
	 this.labelSensor.AutoSize = true;
	 this.labelSensor.Location = new System.Drawing.Point(85, 55);
	 this.labelSensor.Name = "labelSensor";
	 this.labelSensor.Size = new System.Drawing.Size(33, 13);
	 this.labelSensor.TabIndex = 11;
	 this.labelSensor.Text = "None";
	 // 
	 // label15
	 // 
	 this.label15.AutoSize = true;
	 this.label15.Location = new System.Drawing.Point(6, 55);
	 this.label15.Name = "label15";
	 this.label15.Size = new System.Drawing.Size(40, 13);
	 this.label15.TabIndex = 10;
	 this.label15.Text = "Sensor";
	 // 
	 // labelDCAMVer
	 // 
	 this.labelDCAMVer.AutoSize = true;
	 this.labelDCAMVer.Location = new System.Drawing.Point(85, 68);
	 this.labelDCAMVer.Name = "labelDCAMVer";
	 this.labelDCAMVer.Size = new System.Drawing.Size(33, 13);
	 this.labelDCAMVer.TabIndex = 9;
	 this.labelDCAMVer.Text = "None";
	 // 
	 // label13
	 // 
	 this.label13.AutoSize = true;
	 this.label13.Location = new System.Drawing.Point(6, 68);
	 this.label13.Name = "label13";
	 this.label13.Size = new System.Drawing.Size(76, 13);
	 this.label13.TabIndex = 8;
	 this.label13.Text = "DCAM Version";
	 // 
	 // labelCameraModel
	 // 
	 this.labelCameraModel.AutoSize = true;
	 this.labelCameraModel.Location = new System.Drawing.Point(85, 29);
	 this.labelCameraModel.Name = "labelCameraModel";
	 this.labelCameraModel.Size = new System.Drawing.Size(33, 13);
	 this.labelCameraModel.TabIndex = 5;
	 this.labelCameraModel.Text = "None";
	 // 
	 // label8
	 // 
	 this.label8.AutoSize = true;
	 this.label8.Location = new System.Drawing.Point(6, 29);
	 this.label8.Name = "label8";
	 this.label8.Size = new System.Drawing.Size(36, 13);
	 this.label8.TabIndex = 4;
	 this.label8.Text = "Model";
	 // 
	 // labelCameraType
	 // 
	 this.labelCameraType.AutoSize = true;
	 this.labelCameraType.Location = new System.Drawing.Point(85, 42);
	 this.labelCameraType.Name = "labelCameraType";
	 this.labelCameraType.Size = new System.Drawing.Size(33, 13);
	 this.labelCameraType.TabIndex = 3;
	 this.labelCameraType.Text = "None";
	 // 
	 // label6
	 // 
	 this.label6.AutoSize = true;
	 this.label6.Location = new System.Drawing.Point(6, 42);
	 this.label6.Name = "label6";
	 this.label6.Size = new System.Drawing.Size(70, 13);
	 this.label6.TabIndex = 2;
	 this.label6.Text = "Camera Type";
	 // 
	 // labelSerialNum
	 // 
	 this.labelSerialNum.AutoSize = true;
	 this.labelSerialNum.Location = new System.Drawing.Point(85, 16);
	 this.labelSerialNum.Name = "labelSerialNum";
	 this.labelSerialNum.Size = new System.Drawing.Size(55, 13);
	 this.labelSerialNum.TabIndex = 1;
	 this.labelSerialNum.Text = "00000000";
	 // 
	 // label4
	 // 
	 this.label4.AutoSize = true;
	 this.label4.Location = new System.Drawing.Point(6, 16);
	 this.label4.Name = "label4";
	 this.label4.Size = new System.Drawing.Size(73, 13);
	 this.label4.TabIndex = 0;
	 this.label4.Text = "Serial Number";
	 // 
	 // menuStrip1
	 // 
	 this.menuStrip1.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.fileToolStripMenuItem,
            this.cameraToolStripMenuItem});
	 this.menuStrip1.Location = new System.Drawing.Point(0, 0);
	 this.menuStrip1.Name = "menuStrip1";
	 this.menuStrip1.RenderMode = System.Windows.Forms.ToolStripRenderMode.System;
	 this.menuStrip1.Size = new System.Drawing.Size(664, 24);
	 this.menuStrip1.TabIndex = 6;
	 this.menuStrip1.Text = "menuStrip1";
	 // 
	 // fileToolStripMenuItem
	 // 
	 this.fileToolStripMenuItem.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.newCameraToolStripMenuItem,
            this.toolStripSeparator1,
            this.exitToolStripMenuItem});
	 this.fileToolStripMenuItem.Name = "fileToolStripMenuItem";
	 this.fileToolStripMenuItem.Size = new System.Drawing.Size(35, 20);
	 this.fileToolStripMenuItem.Text = "File";
	 // 
	 // newCameraToolStripMenuItem
	 // 
	 this.newCameraToolStripMenuItem.Name = "newCameraToolStripMenuItem";
	 this.newCameraToolStripMenuItem.Size = new System.Drawing.Size(135, 22);
	 this.newCameraToolStripMenuItem.Text = "New Camera";
	 this.newCameraToolStripMenuItem.Click += new System.EventHandler(this.newCameraToolStripMenuItem_Click);
	 // 
	 // toolStripSeparator1
	 // 
	 this.toolStripSeparator1.Name = "toolStripSeparator1";
	 this.toolStripSeparator1.Size = new System.Drawing.Size(132, 6);
	 // 
	 // exitToolStripMenuItem
	 // 
	 this.exitToolStripMenuItem.Name = "exitToolStripMenuItem";
	 this.exitToolStripMenuItem.Size = new System.Drawing.Size(135, 22);
	 this.exitToolStripMenuItem.Text = "Exit";
	 this.exitToolStripMenuItem.Click += new System.EventHandler(this.exitToolStripMenuItem_Click);
	 // 
	 // cameraToolStripMenuItem
	 // 
	 this.cameraToolStripMenuItem.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.saveImageToolStripMenuItem,
            this.toolStripSeparator2,
            this.cameraControlDialogToolStripMenuItem});
	 this.cameraToolStripMenuItem.Name = "cameraToolStripMenuItem";
	 this.cameraToolStripMenuItem.Size = new System.Drawing.Size(56, 20);
	 this.cameraToolStripMenuItem.Text = "Camera";
	 // 
	 // saveImageToolStripMenuItem
	 // 
	 this.saveImageToolStripMenuItem.Name = "saveImageToolStripMenuItem";
	 this.saveImageToolStripMenuItem.Size = new System.Drawing.Size(181, 22);
	 this.saveImageToolStripMenuItem.Text = "Save Image";
	 this.saveImageToolStripMenuItem.Click += new System.EventHandler(this.saveImageToolStripMenuItem_Click);
	 // 
	 // toolStripSeparator2
	 // 
	 this.toolStripSeparator2.Name = "toolStripSeparator2";
	 this.toolStripSeparator2.Size = new System.Drawing.Size(178, 6);
	 // 
	 // cameraControlDialogToolStripMenuItem
	 // 
	 this.cameraControlDialogToolStripMenuItem.Name = "cameraControlDialogToolStripMenuItem";
	 this.cameraControlDialogToolStripMenuItem.Size = new System.Drawing.Size(181, 22);
	 this.cameraControlDialogToolStripMenuItem.Text = "Camera Control Dialog";
	 this.cameraControlDialogToolStripMenuItem.Click += new System.EventHandler(this.cameraControlDialogToolStripMenuItem_Click);
	 // 
	 // statusStrip1
	 // 
	 this.statusStrip1.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.toolStripStatusLabel1});
	 this.statusStrip1.Location = new System.Drawing.Point(0, 690);
	 this.statusStrip1.Name = "statusStrip1";
	 this.statusStrip1.Size = new System.Drawing.Size(664, 22);
	 this.statusStrip1.TabIndex = 7;
	 this.statusStrip1.Text = "statusStrip1";
	 // 
	 // toolStripStatusLabel1
	 // 
	 this.toolStripStatusLabel1.Name = "toolStripStatusLabel1";
	 this.toolStripStatusLabel1.Size = new System.Drawing.Size(94, 17);
	 this.toolStripStatusLabel1.Text = "Image information";
	 // 
	 // checkBoxAutoResize
	 // 
	 this.checkBoxAutoResize.AutoSize = true;
	 this.checkBoxAutoResize.Checked = true;
	 this.checkBoxAutoResize.CheckState = System.Windows.Forms.CheckState.Checked;
	 this.checkBoxAutoResize.Location = new System.Drawing.Point(509, 39);
	 this.checkBoxAutoResize.Name = "checkBoxAutoResize";
	 this.checkBoxAutoResize.Size = new System.Drawing.Size(83, 17);
	 this.checkBoxAutoResize.TabIndex = 8;
	 this.checkBoxAutoResize.Text = "Auto Resize";
	 this.checkBoxAutoResize.UseVisualStyleBackColor = true;
	 this.checkBoxAutoResize.CheckedChanged += new System.EventHandler(this.checkBoxAutoResize_CheckedChanged);
	 // 
	 // checkBoxDrawShapes
	 // 
	 this.checkBoxDrawShapes.AutoSize = true;
	 this.checkBoxDrawShapes.Location = new System.Drawing.Point(509, 62);
	 this.checkBoxDrawShapes.Name = "checkBoxDrawShapes";
	 this.checkBoxDrawShapes.Size = new System.Drawing.Size(90, 17);
	 this.checkBoxDrawShapes.TabIndex = 9;
	 this.checkBoxDrawShapes.Text = "Draw Shapes";
	 this.checkBoxDrawShapes.UseVisualStyleBackColor = true;
	 this.checkBoxDrawShapes.CheckedChanged += new System.EventHandler(this.checkBoxDrawShapes_CheckedChanged);
	 // 
	 // ActiveFlyCapForm
	 // 
	 this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
	 this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
	 this.AutoSizeMode = System.Windows.Forms.AutoSizeMode.GrowAndShrink;
	 this.ClientSize = new System.Drawing.Size(664, 712);
	 this.Controls.Add(this.checkBoxDrawShapes);
	 this.Controls.Add(this.checkBoxAutoResize);
	 this.Controls.Add(this.statusStrip1);
	 this.Controls.Add(this.axActiveFlyCapControl);
	 this.Controls.Add(this.groupBox2);
	 this.Controls.Add(this.groupBox1);
	 this.Controls.Add(this.buttonStartStop);
	 this.Controls.Add(this.buttonQuit);
	 this.Controls.Add(this.buttonCamControlDlg);
	 this.Controls.Add(this.menuStrip1);
	 this.DoubleBuffered = true;
	 this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
	 this.MainMenuStrip = this.menuStrip1;
	 this.MaximizeBox = false;
	 this.Name = "ActiveFlyCapForm";
	 this.Text = "ActiveFlyCapCSharpEx";
	 this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.Form1_FormClosing);
	 ((System.ComponentModel.ISupportInitialize)(this.axActiveFlyCapControl)).EndInit();
	 this.groupBox1.ResumeLayout(false);
	 this.groupBox1.PerformLayout();
	 this.groupBox2.ResumeLayout(false);
	 this.groupBox2.PerformLayout();
	 this.menuStrip1.ResumeLayout(false);
	 this.menuStrip1.PerformLayout();
	 this.statusStrip1.ResumeLayout(false);
	 this.statusStrip1.PerformLayout();
	 this.ResumeLayout(false);
	 this.PerformLayout();

      }

      #endregion

      private AxActiveFlyCapLib.AxActiveFlyCapControl axActiveFlyCapControl;
      private System.Windows.Forms.Button buttonStartStop;
      private System.Windows.Forms.Button buttonCamControlDlg;
      private System.Windows.Forms.GroupBox groupBox1;
      private System.Windows.Forms.Label label1;
      private System.Windows.Forms.TextBox textBox2;
      private System.Windows.Forms.TextBox textBox1;
      private System.Windows.Forms.Label label3;
      private System.Windows.Forms.Button buttonSetRegister;
      private System.Windows.Forms.Button buttonGetRegister;
      private System.Windows.Forms.Label label2;
      private System.Windows.Forms.Button buttonQuit;
      private System.Windows.Forms.GroupBox groupBox2;
      private System.Windows.Forms.Label labelSensor;
      private System.Windows.Forms.Label label15;
      private System.Windows.Forms.Label labelDCAMVer;
      private System.Windows.Forms.Label label13;
      private System.Windows.Forms.Label labelCameraModel;
      private System.Windows.Forms.Label label8;
      private System.Windows.Forms.Label labelCameraType;
      private System.Windows.Forms.Label label6;
      private System.Windows.Forms.Label labelSerialNum;
      private System.Windows.Forms.Label label4;
      private System.Windows.Forms.Label labelBusSpeed;
      private System.Windows.Forms.Label label21;
      private System.Windows.Forms.Label labelBusNode;
      private System.Windows.Forms.Label label23;
      private System.Windows.Forms.MenuStrip menuStrip1;
      private System.Windows.Forms.ToolStripMenuItem fileToolStripMenuItem;
      private System.Windows.Forms.ToolStripMenuItem newCameraToolStripMenuItem;
      private System.Windows.Forms.ToolStripSeparator toolStripSeparator1;
      private System.Windows.Forms.ToolStripMenuItem exitToolStripMenuItem;
      private System.Windows.Forms.ToolStripMenuItem cameraToolStripMenuItem;
      private System.Windows.Forms.ToolStripMenuItem saveImageToolStripMenuItem;
      private System.Windows.Forms.ToolStripSeparator toolStripSeparator2;
      private System.Windows.Forms.ToolStripMenuItem cameraControlDialogToolStripMenuItem;
      private System.Windows.Forms.StatusStrip statusStrip1;
      private System.Windows.Forms.ToolStripStatusLabel toolStripStatusLabel1;
      private System.Windows.Forms.CheckBox checkBoxAutoResize;
      private System.Windows.Forms.CheckBox checkBoxDrawShapes;




   }
}

