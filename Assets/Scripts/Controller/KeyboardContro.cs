using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class KeyboardContro : MonoBehaviour
{
    public Text text_;
    private Contro.ControKeyCode KeyHasTriggered;
    // Start is called before the first frame update
    void Start()
    {
        Contro._KeyCode = 0;
    }

    // Update is called once per frame
    void Update()
    {
        HandleKeyInput(KeyCode.W, Contro.ControKeyCode.translationFront);
        HandleKeyInput(KeyCode.A, Contro.ControKeyCode.translationLeft);
        HandleKeyInput(KeyCode.S, Contro.ControKeyCode.translationBack);
        HandleKeyInput(KeyCode.D, Contro.ControKeyCode.translationRight);

        HandleKeyInput(KeyCode.I, Contro.ControKeyCode.rotationUp);
        HandleKeyInput(KeyCode.K, Contro.ControKeyCode.rotationDown);
        HandleKeyInput(KeyCode.J, Contro.ControKeyCode.rotationLeft);
        HandleKeyInput(KeyCode.L, Contro.ControKeyCode.rotationRight);
        
        HandleKeyInput(KeyCode.U, Contro.ControKeyCode.YawRotateLeft);
        HandleKeyInput(KeyCode.O, Contro.ControKeyCode.YawRotateRight);

        HandleKeyInput(KeyCode.Space, Contro.ControKeyCode.heightUp);
        HandleKeyInput(KeyCode.LeftControl, Contro.ControKeyCode.heightDown);

        HandleKeyInput(KeyCode.LeftShift, Contro.ControKeyCode.speed_state);

        HandleKeyInput(KeyCode.X, Contro.ControKeyCode.rotateMode);

        HandleKeyInput(KeyCode.R, Contro.ControKeyCode.ResetFlag);

        HandleKeyInput(KeyCode.Z, Contro.ControKeyCode.EnableChangeDecodeFunc);
        HandleKeyInput_Switch(KeyCode.B, Contro.ControKeyCode.EnableDecodeMode);
        HandleKeyInput_Switch(KeyCode.N, Contro.ControKeyCode.EnableGenerateTarget);
        HandleKeyInput_Switch(KeyCode.C, Contro.ControKeyCode.translateMode);
        HandleKeyInput_Switch(KeyCode.T, Contro.ControKeyCode.CustomPositionContro);
        HandleKeyInput_Switch(KeyCode.Y, Contro.ControKeyCode.CustomTargetContro);

        bool TranslateMode,RotateMode;
        float TranslateFront_Back, TranslateLeft_Right;
        float RotateUp_Down, RotateLeft_Right, YawRotation;
        
        TranslateMode = Input.GetAxis("Key_1")==1;
        TranslateFront_Back = -Input.GetAxis("Vertical_L") / 1000.0f;
        TranslateLeft_Right = Input.GetAxis("Horizontal_L") / 1000.0f;
        ControTargetTranslation(TranslateFront_Back, TranslateLeft_Right, TranslateMode);

        RotateMode = Input.GetAxis("Key_2")==1;
        RotateUp_Down = -Input.GetAxis("Vertical_R") * 2.0f;
        RotateLeft_Right = Input.GetAxis("Horizontal_R") * 2.0f;
        YawRotation = 0;
        ControTargetRotation(RotateUp_Down, RotateLeft_Right, YawRotation, RotateMode);

        if (Input.GetAxis("Key_3")==1)
            Contro.ArmControMode = Contro.ArmControMode_.ArmReset;
        text_.text = Contro.ArmControMode.ToString();
            
        // YawRotation = Input.GetAxis("Horizontal_R") / 100.0f;

    }

    void HandleKeyInput(KeyCode key, Contro.ControKeyCode stateFlag)
    {
        if (Input.GetKey(key))
        {
            Contro._KeyCode |= stateFlag; // Set the bit
        }
        else if (!Input.GetKey(key))
        {
            Contro._KeyCode &= ~stateFlag; // Clear the bit
        }
    }

    void HandleKeyInput_Switch(KeyCode key, Contro.ControKeyCode stateFlag)
    {
        if (Input.GetKeyDown(key))
        {
            if (Contro.GetControKeyCode(stateFlag))
            {
                Contro._KeyCode &= ~stateFlag;
            }
            else
            {
                Contro._KeyCode |= stateFlag;
            }
        }
    }
    
    void ControTargetTranslation(float Front_Back, float Left_Right, bool SwitchToWordCoordinate)
    {
        Vector3 Translation = new Vector3(Left_Right, 0, Front_Back);
        if(SwitchToWordCoordinate)
            Arm.TargetTranslate_baseWorld(Translation);
        else
            Arm.TargetTranslate_baseSucker(Translation);
        Contro.ArmControMode = Contro.ArmControMode_.TargetDecode;
    }
    void ControTargetRotation(float Up_Down, float Left_Right,float YawRotation, bool SwitchRotateMode)
    {
        Vector3 Rotate = new Vector3(Up_Down, Left_Right, 0);
        Vector3 YawRotate = new Vector3(0, YawRotation, 0);
        if(SwitchRotateMode)
        {
            Arm.TargetRotate_Joint(Up_Down, Left_Right);
            Arm.AssortedYawRotate(YawRotate);
            
            Contro.ArmControMode = Contro.ArmControMode_.ArmGenerate;
        }
        else
        {
            Arm.TargetRotate(Rotate);
            Arm.BigYawRotate(YawRotate);
            
            Contro.ArmControMode = Contro.ArmControMode_.TargetDecode;
        }
    }
}

