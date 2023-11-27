{..............................................................................}
{ Summary Creating different PCB Objects                                       }
{                                                                              }
{    CreateStrings procedure creates two different text objects on a new PCB   }
{    PlacePCBObjects procedure creates different objects on a new PCB          }
{                                                                              }
{ Copyright (c) 2007 by Altium Limited                                         }
{..............................................................................}


{..............................................................................}
Var
    Board     : IPCB_Board;
    WorkSpace : IWorkSpace;

{..............................................................................}
Procedure PlaceAPCBText(inputtext , X , Y );
Var
    TextObj : IPCB_Text;
Begin
    (* create a text object on a top overlay *)
    Board.LayerIsDisplayed[eTopOverLay] := True;

    TextObj := PCBServer.PCBObjectFactory(eTextObject, eNoDimension, eCreate_Default);
    TextObj.XLocation := MMsToCoord(X);
    TextObj.YLocation := MMsToCoord(Y);
    TextObj.Layer     := eTopOverlay;
    TextObj.Text      := inputtext;
    TextObj.Size      := MMsToCoord(2);   // sets the height of the text.
    TextObj.UseTTFonts := True;
    TextObj.FontName  := 'Times New Roman';
    Board.AddPCBObject(TextObj);
End;
{..............................................................................}
{..............................................................................}
Procedure PlaceAPCBVia(X , Y , HOLE , PAD);
Var
    Via         : IPCB_Via;
    PadCache    : TPadCache;
Begin
    (* Create a Via object*)
    Via           := PCBServer.PCBObjectFactory(eViaObject, eNoDimension, eCreate_Default);
    Via.X         := MMsToCoord(X);
    Via.Y         := MMsToCoord(Y);
    Via.Size      := MMsToCoord(PAD);
    Via.HoleSize  := MMsToCoord(HOLE);
    Via.LowLayer  := eTopLayer;
    Via.HighLayer := eBottomLayer;
    Via.Net       := 1 ;
    (* Setup a pad cache *)
    Padcache := Via.GetState_Cache;
    Padcache.ReliefAirGap := MMsToCoord(0.1);
    Padcache.PowerPlaneReliefExpansion := MMsToCoord(0.1);
    Padcache.PowerPlaneClearance       := MMsToCoord(0.1);
    Padcache.ReliefConductorWidth      := MMsToCoord(0.1);
    Padcache.SolderMaskExpansion       := MMsToCoord(0.1);
    Padcache.SolderMaskExpansionValid  := eCacheManual;
    Padcache.PasteMaskExpansion        := MMsToCoord(0.1);
    Padcache.PasteMaskExpansionValid   := eCacheManual;

    (* Assign the new pad cache to the via *)
    Via.SetState_Cache                 := Padcache;

    Board.AddPCBObject(Via);
End;
{..............................................................................}

{..............................................................................}
Procedure PlaceAPCBTrack(X1, Y1, X2, Y2, Layer, Net , Width);
Var
    Track         : IPCB_Track;
Begin
    (* Create a Track object*)
    Track             := PCBServer.PCBObjectFactory(eTrackObject, eNoDimension, eCreate_Default);
    Track.X1          := MMsToCoord(X1);
    Track.Y1          := MMsToCoord(Y1);
    Track.X2          := MMsToCoord(X2);
    Track.Y2          := MMsToCoord(Y2);
    Track.Layer       := Layer;
    Track.Width       := MMsToCoord(Width);
    Track.Net         := Net;
    Board.AddPCBObject(Track);
End;
{..............................................................................}
Procedure coilSpiralCW(OD , ID , W , S, X0 ,Y0 ,Layer , net);
Var
    I         : Integer;
    N         : double;
    K         : Integer;
    L         : Double;
    X1        : Double;
    Y1        : Double;
    X2        : Double;
    Y2        : Double;
Begin
    N := 4 * ( ( OD - ID + S )/(2.0 * ( W + S )) );
    L := OD;
    X2 := X0 - L / 2;
    Y2 := Y0 + L / 2;

    For I := 1 To N Do
    Begin
       K := I mod 4;
       Case K of
            1 : Begin
                X1 := X2;
                Y1 := Y2;
                X2 := X1 + L;
                Y2 := Y1;
                End;
            2 : Begin
                L := L - w - s;
                 X1 := X2;
                Y1 := Y2;
                X2 := X1;
                Y2 := Y1 - L ;
                End;
            3 : Begin
                 X1 := X2;
                Y1 := Y2;
                X2 := X1 - L;
                Y2 := Y1 ;
                End;
            0 : Begin
                L := L - w - s;
                 X1 := X2;
                Y1 := Y2;
                X2 := X1;
                Y2 := Y1 + L ;
                End;
       End;
       PlaceAPCBTrack(X1,Y1,X2,Y2,Layer,net,W);
    End;
    k := n div 4  ;
    PlaceAPCBText(format('OD=%d,ID=%d,n=%d',[OD,ID,k]),X0-ID/2,Y0);
End;
{..............................................................................}
Procedure coilRECT_CW(L , H , N , W , S, X0 ,Y0 ,mode , Layer , net);
Var
    I         : Integer;
    K         : Integer;
    X1        : Double;
    Y1        : Double;
    X2        : Double;
    Y2        : Double;
Begin
     case mode of
          1: begin
             X2 := X0 -  L / 2 ;
             Y2 := Y0 + H / 2;
             end  ;
          2: begin
             X2 := X0 + L / 2;
             Y2 := Y0 + H / 2 ;
             end  ;
          3: begin
             X2 := X0 + L / 2 ;
             Y2 := Y0 - H / 2;
             end ;
          4: begin
             X2 := X0 - L / 2;
             Y2 := Y0 - H / 2 ;
             end  ;
     end;

    N := N * 4;
    For I := mode  To( N + mode - 1)  Do
    Begin
       K := I mod 4;
       Case K of
            1 : Begin
                X1 := X2;
                Y1 := Y2;
                X2 := X1 + L;
                Y2 := Y1;
                if I = mode   then
                    L := L
                else  L := L - w - s;
                End;
            2 : Begin
                X1 := X2;
                Y1 := Y2;
                X2 := X1;
                Y2 := Y1 - H ;
                if I = mode   then
                    H := H
                else  H := H - w - s;
                End;
            3 : Begin
                X1 := X2;
                Y1 := Y2;
                X2 := X1 - L;
                Y2 := Y1 ;
                if I =  mode  then
                    L := L
                else  L := L - w - s;
                End;
            0 : Begin
                X1 := X2;
                Y1 := Y2;
                X2 := X1;
                Y2 := Y1 + H ;
                if I = mode   then
                    H := H
                else  H := H - w - s;
                End;
       End;
       PlaceAPCBTrack(X1,Y1,X2,Y2,Layer,net,W);
    End;
    //PlaceAPCBText(format('Width=%d,height=%d,n=%d',[L,H,N]),X0,Y0);
End;
{..............................................................................}
Procedure coilRECT_CCW(L , H , N , W , S, X0 ,Y0 , mode ,Layer , net);
Var
    I         : Integer;
    K         : Integer;
    X1        : Double;
    Y1        : Double;
    X2        : Double;
    Y2        : Double;
Begin
    case mode of
          1: begin
             X2 := X0 - L / 2;
             Y2 := Y0 + H / 2;
             end  ;
          2: begin
             mode := 4;
             X2 := X0 + L / 2;
             Y2 := Y0 + H / 2;
             end  ;
          3: begin
             X2 := X0 + L / 2;
             Y2 := Y0 - H / 2;
             end ;
          4: begin
             mode := 2;
             X2 := X0 - L / 2;
             Y2 := Y0 - H / 2;
             end  ;
     end;
    N := N * 4;

    For I := mode  To( N + mode - 1)  Do
    Begin
       K := I mod 4;
       Case K of
            0 : Begin
                X1 := X2;
                Y1 := Y2;
                X2 := X1 - L;
                Y2 := Y1;
                if I = mode   then
                    L := L
                else  L := L - w - s;
                End;
            1 : Begin
                 X1 := X2;
                Y1 := Y2;
                X2 := X1;
                Y2 := Y1 - H ;
                if I = mode   then
                    H := H
                else  H := H - w - s;
                End;
            2 : Begin
                 X1 := X2;
                Y1 := Y2;
                X2 := X1 + L;
                Y2 := Y1 ;
                if I = mode   then
                    L := L
                else  L := L - w - s;
                End;
            3 : Begin
                 X1 := X2;
                Y1 := Y2;
                X2 := X1;
                Y2 := Y1 + H ;
                if I = mode   then
                    H := H
                else  H := H - w - s;
                End;
       End;
       PlaceAPCBTrack(X1,Y1,X2,Y2,Layer,net,W);
    End;
    //PlaceAPCBText(format('Width=%d,height=%d,n=%d',[L,H,N]),X0,Y0);
End;
{..............................................................................}
Procedure coilSpiralCCW(OD , ID , W , S, X0 ,Y0,Layer, net);
Var
    I         : Integer;
    N         : double;
    K         : Integer;
    L         : Double;
    X1        : Double;
    Y1        : Double;
    X2        : Double;
    Y2        : Double;
Begin
    N := 4 * ( ( OD - ID + S )/(2.0 * ( W + S )) );
    L := OD;
    X2 := X0 + L / 2;
    Y2 := Y0 + L / 2;

    For I := 1 To N Do
    Begin
       K := I mod 4;
       Case K of
            1 : Begin
                X1 := X2;
                Y1 := Y2;
                X2 := X1 - L;
                Y2 := Y1;
                End;
            2 : Begin
                L := L - w - s;
                 X1 := X2;
                Y1 := Y2;
                X2 := X1;
                Y2 := Y1 - L ;
                End;
            3 : Begin
                 X1 := X2;
                Y1 := Y2;
                X2 := X1 + L;
                Y2 := Y1 ;
                End;
            0 : Begin
                L := L - w - s;
                 X1 := X2;
                Y1 := Y2;
                X2 := X1;
                Y2 := Y1 + L ;
                End;
       End;
       PlaceAPCBTrack(X1,Y1,X2,Y2,Layer,net,W);
    End;
     k := n div 4  ;
    PlaceAPCBText(format('OD=%d,ID=%d,n=%d',[OD,ID,k]),X0-ID/2,Y0);
End;
{..............................................................................}
Procedure coilHexagonCW(OD , ID , W , S, X0 ,Y0 ,Layer , net);
Var
    I         : Integer;
    N         : double;
    K         : Integer;
    L         : Double;
    X1        : Double;
    Y1        : Double;
    X2        : Double;
    Y2        : Double;
const
Alpha = 0.8660254;
Cot30 = 1.7320508;

Begin
    N := Round(( Alpha * ( OD - ID )) / ( 2.0 * ( W + S ))) + 1 ;
    N := N * 6;
    L := OD / 2;

    For I := 1 To N Do
    Begin
       K := I mod 6;
       Case K of
            1 : Begin
                if I = 1 then
                   X1 := X0 - L / 2
                else
                     X1 := X0 - L / 2  - ( w + s ) / Alpha;
                Y1 := Y0 + L  * Alpha;
                X2 := X0 + L / 2;
                Y2 := Y0 + L  * Alpha;
                End;
            2 : Begin
                X1 := X0 + L / 2;
                Y1 := Y0 + L  * Alpha;
                X2 := X0 + L;
                Y2 := Y0 ;
                End;
            3 : Begin
                X1 := X0 + L ;
                Y1 := Y0 ;
                X2 := X0 + L / 2;
                Y2 := Y0 - L  * Alpha;
                End;
            4 : Begin
                X1 := X0 + L / 2;
                Y1 := Y0 - L  * Alpha;
                X2 := X0 - L / 2;
                Y2 := Y0 - L  * Alpha;
                End;
            5 : Begin
                X1 := X0 - L / 2;
                Y1 := Y0 - L  * Alpha;
                X2 := X0 - L ;
                Y2 := Y0;
                End;
            0 : Begin
                X1 := X0 - L ;
                Y1 := Y0 ;
                X2 := X0 - L / 2 - (w+s)/Cot30;
                Y2 := Y0 + L  * Alpha  - (w+s);
                L := L - 2 *( w + s) / cot30;
                End;
       End;
       PlaceAPCBTrack(X1,Y1,X2,Y2,Layer,net,W);
    End;
    k := n div 6  ;
    PlaceAPCBTrack(X0-5,Y0,X0+5,Y0,eTopOverLay,0,0.254);
    PlaceAPCBTrack(X0,Y0-5,X0,Y0+5,eTopOverLay,0,0.254);
    PlaceAPCBText(format('OD=%d,ID=%d,n=%d',[OD,ID,k]),X0-15,Y0+8);
End;
{..............................................................................}
Procedure coilNside(N , OD ,ID , W , S, X0 ,Y0 ,CCW , Rotation ,Layer , net);
Var
    I         : Integer;
    j         : Integer;
    Nr        : Integer;
    K         : Integer;
    L         : Double;
    R         : Double;
    X1        : Double;
    Y1        : Double;
    X2        : Double;
    Y2        : Double;
    Xt       : Double;
    Yt       : Double;
    theta     : Double;
const
pi = 3.14159265;
Begin
    Nr := Round(( cos(pi/N) * ( OD - ID )) / ( 2.0 * ( W + S ))) + 1 ;
    R := OD / 2;

    For I := 1 To Nr Do
    Begin
       For j := 0 To N-1 Do
       begin
           theta :=   j * 2*pi / N * (1-2*CCW) + Rotation;
           if j = 0 then
                      begin
                      X1 :=  R * cos(theta- Rotation) + (w+s)/sin(pi*(N-2)/2/N)/2;
                      Y1 :=  R * sin(theta- Rotation) - (w+s)/cos(pi*(N-2)/2/N)/2*(1-2*CCW);
                      Xt := X1 * cos(Rotation) - Y1 * sin(Rotation);
                      Yt := X1 * sin(Rotation) + Y1 * cos(Rotation);
                      X1 := Xt + X0;
                      Y1 := Yt + Y0;
                      end
           else
                      begin
                      X1 := X0 +  R * cos(theta);
                      Y1 := Y0 +  R * sin(theta);
           end ;
           theta :=   ( j + 1 ) * 2*pi / N * (1-2*CCW)+ Rotation;
           if j = N-1 then
                      begin
                      X2 :=  R * cos(theta- Rotation) - (w+s)/sin(pi*(N-2)/2/N)/2 ;
                      Y2 :=  R * sin(theta- Rotation) - (w+s)/cos(pi*(N-2)/2/N)/2 *(1-2*CCW);
                      Xt := X2 * cos(Rotation) - Y2 * sin(Rotation);
                      Yt := X2 * sin(Rotation) + Y2 * cos(Rotation);
                      X2 := Xt + X0;
                      Y2 := Yt + Y0;
                      end
           else
                      begin
                      X2 := X0 +  R * cos(theta);
                      Y2 := Y0 +  R * sin(theta);
           end  ;
           PlaceAPCBTrack(X1,Y1,X2,Y2,Layer,net,W);
       End;
       R := R - ( w + s ) /cos(pi/n);
    End;
    PlaceAPCBTrack(X0-5,Y0,X0+5,Y0,eTopOverLay,0,0.254);
    PlaceAPCBTrack(X0,Y0-5,X0,Y0+5,eTopOverLay,0,0.254);
    PlaceAPCBText(format('OD=%d,ID=%d,n=%d',[OD,ID,Nr]),X0-15,Y0+8);
End;
{..............................................................................}
Procedure coilSpiral(ID ,N , w , s, dr ,X0 ,Y0 ,CCW , Rotation ,Layer , net);
Var
    I         : Integer;
    j         : Integer;
    Nt         : Integer;
    L         : Double;
    R         : Double;
    Rin         : Double;
    X1        : Double;
    Y1        : Double;
    X2        : Double;
    Y2        : Double;
    Xt       : Double;
    Yt       : Double;
    theta     : Double;
Begin
    Rin := ID / 2;
    R := Rin;
    theta := Rotation;
    For I := 0 To N-1 Do
    begin
        //Nt := 2*pi*R/dr;
        j:=0;
        while (theta- Rotation)*(1-2*CCW) <= 2*pi*N  do
        begin
             theta := (j * dr / R + I * 2 *pi)*(1-2*CCW) + Rotation;
            R := Rin + (w + s)/2/pi* theta*(1-2*CCW);
            X1 := X0 +  R * cos(theta);
            Y1 := Y0 +  R * sin(theta);
            theta := ((j+1) * dr / R + I * 2 *pi)*(1-2*CCW) + Rotation;
            R := Rin + (w + s)/2/pi* theta*(1-2*CCW);
            X2 := X0 +  R * cos(theta);
            Y2 := Y0 +  R * sin(theta);
            PlaceAPCBTrack(X1,Y1,X2,Y2,Layer,net,W);
            j := j +1;
        end ;
        theta := 0;
    End;
    PlaceAPCBTrack(X0-5,Y0,X0+5,Y0,eTopOverLay,0,0.254);
    PlaceAPCBTrack(X0,Y0-5,X0,Y0+5,eTopOverLay,0,0.254);
    //PlaceAPCBText(format('Nt=%d,N=%d',[Nt,N]),X0-15,Y0+8);
End;
{..............................................................................}
Procedure polyNside(N , D , W , X0 ,Y0 , Rotation ,Layer , net);
Var
    j         : Integer;
    K         : Integer;
    R         : Double;
    X1        : Double;
    Y1        : Double;
    X2        : Double;
    Y2        : Double;
    theta     : Double;
const
pi = 3.14159265;
Begin
    R := D / 2;
       For j := 0 To N-1 Do
       begin
           theta :=   j * 2*pi / N  + Rotation;
           X1 := X0 +  R * cos(theta);
           Y1 := Y0 +  R * sin(theta);
           theta :=   ( j + 1 ) * 2*pi / N + Rotation;
           X2 := X0 +  R * cos(theta);
           Y2 := Y0 +  R * sin(theta);
           PlaceAPCBTrack(X1,Y1,X2,Y2,Layer,net,W);
       End;
End;
{..............................................................................}
Procedure PlaceAPCBArc(Dummy : Integer);
Var
    Arc : IPCB_Arc;
Begin
    Arc := PCBServer.PCBObjectFactory(eArcObject, eNoDimension, eCreate_Default);
    Arc.XCenter    := MilsToCoord(Board.XOrigin + 1800);
    Arc.YCenter    := MilsToCoord(Board.YOrigin + 1800);
    Arc.Radius     := MilsToCoord(200);
    Arc.LineWidth  := MilsToCoord(50);
    Arc.StartAngle := 0;
    Arc.EndAngle   := 270;
    Arc.Layer      := eBottomLayer;
    Board.AddPCBObject(Arc);
End;
{..............................................................................}

{..............................................................................}
Procedure PlacePCBObjects;
Var
 CCW : Integer;
 L :  Integer;
 H :  Integer;
 w :  Integer;
 N :  Integer;
 s :  Integer;
 dis  : Integer;
Begin
    (*create a new pcb document *)
    WorkSpace := GetWorkSpace;
    If WorkSpace = Nil Then Exit;
    Workspace.DM_CreateNewDocument('PCB');

    If PCBServer = Nil Then Exit;
    Board := PCBServer.GetCurrentPCBBoard;
    If Board = Nil then exit;
    dis := 60;
    (* Place new PCB objects*)
     coilSpiralCW(50 , 5 , 0.9 , 0.3, dis , dis,eTopLayer,1);
     coilSpiralCCW(50 , 15 , 1.2 , 0.25, dis * 2 ,dis,eBottomLayer,1);
     coilHexagonCW(50 , 21 , 1 , 0.35, dis  * 3,dis ,eTopLayer ,1);
     CCW := 0;
     coilNside(4, 50 ,15 , 1 , 0.35, dis  * 4,dis ,CCW,Pi/4,eTopLayer , 1);
     coilSpiral(15 ,10 , 1 , 0.35, 1 ,dis  * 5,dis ,CCW , 0 ,eTopLayer , 1);
     CCW := 1;
     //coilNside(4, 50 ,15 , 1 , 0.35, 100 ,50 ,CCW,3*Pi/4,eTopLayer , 1);
     //coilNside(30, 100 ,50 , 1 , 0.35, 200 ,50 ,CCW,PI/2,eTopLayer , 1);
     coilNside(5, 50 ,15 , 1 , 0.35, dis  ,dis * 2 ,CCW,7*pi/10,eTopLayer , 1);
     //coilNside(4, 50 ,15 , 1 , 0.35, 100 ,150 ,CCW,3*pi/4,eTopLayer , 1);
     //PlaceAPCBVia( 100 , 150 , 1 , 1.5);
     L := 50;
     N := 8;
     w := 1;
     s := 0.35;
     H := ( L - ( w + s ))/2 ;
     coilRECT_CW(L , H , N , w , s , dis * 2 ,dis * 2 , 1, eTopLayer , 1);
     coilRECT_CW(L , H , N , w , s , dis * 2 ,dis * 2 + H  + w + s  , 4 ,eTopLayer , 1);

     coilRECT_CCW(H , L , N , w , s, dis * 2 - (H+w+s)/2 ,dis * 2 + (H+w+s)/2 , 2 ,eBottomLayer , 1);
     coilRECT_CCW(H , L , N , w , s, dis * 2 - (H+w+s)/2 + H  + w + s ,dis * 2 + (H+w+s)/2, 1 ,eBottomLayer , 1);


     L := 30;
     N := 3;
     w := 3;
     coilRECT_CW(L , L , N , w , s , dis * 3 ,dis * 2 , 1, eTopLayer , 1);
     //coilRECT_CCW(L , L , N , w , s , 100 ,100 , 1, eBottomLayer , 1);
     //PlaceAPCBVia( 100 , 100 , 1 , 1.5);
     //coilNside(7, 50 ,15 , 1 , 0.35, 150 ,150 ,CCW,9*pi/14,eTopLayer , 1);
     //coilNside(8, 50 ,15 , 1 , 0.35, 200 ,150 ,CCW,5*pi/8,eTopLayer , 1);
     //coilSpiral(15 ,13 , 1 , 0.35, 1 ,200 ,100 ,CCW , pi/2 ,eTopLayer , 1);
     //polyNside(6 , 6 , 0.3 , 50 ,50 , 0 , eTopLayer , 0);

    (* Refresh PCB workspace *)
    ResetParameters;
    AddStringParameter('Action', 'All');
    RunProcess('PCB:Zoom');
End;
{..............................................................................}

{..............................................................................}

