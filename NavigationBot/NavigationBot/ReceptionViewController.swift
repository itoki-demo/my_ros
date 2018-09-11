//
//  ReceptionViewController.swift
//  NavigationBot
//
//  Created by 浜田 on 2018/09/03.
//  Copyright © 2018 swiftbigg. All rights reserved.
//

import UIKit

class ReceptionViewController: UIViewController {

    //予約情報の構造体
    struct bookInfoJson: Codable{
        let b_ID: String?//社員ID
        let b_Name: String? //予約者名
        let b_RoomName: String? //部屋名
        let b_Date: String? //日程
        let b_StartTime: String?
        let b_FinishTime: String?
        let b_Time: String? //利用時間(分)
        let b_Num: String? //予約番号
    }
    var b_Info:bookInfoJson?
    
    var cautionFlag:Bool?
    
    override func viewDidLoad() {
        super.viewDidLoad()
        deleteUpdate()
        print(IPAddress)
        IPAddress = readIPAddress()
        cautionFlag = false
        
        // Do any additional setup after loading the view.
    }
    
    func readIPAddress() -> String{
        let file_name = "IPAddress.txt"
        
        if let dir = FileManager.default.urls( for: .documentDirectory, in: .userDomainMask ).first {
            
            let path_file_name = dir.appendingPathComponent( file_name )
            do {
                let text = try String( contentsOf: path_file_name, encoding: String.Encoding.utf8 )
                return text
            } catch {
                
            }
        }
        return ""
    }

    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }
    
    
    @IBOutlet weak var Input1Label: UILabel!
    @IBOutlet weak var Input2Label: UILabel!
    @IBOutlet weak var Input3Label: UILabel!
    @IBOutlet weak var Input4Label: UILabel!
    
    @IBAction func Input0ButtonAction(_ sender: Any) {
        inputUpdate(Num: "0")
    }
    @IBAction func Input1ButtonAction(_ sender: Any) {
        inputUpdate(Num: "1")
    }
    @IBAction func Input2ButtonAction(_ sender: Any) {
        inputUpdate(Num: "2")
    }
    @IBAction func Input3ButtonAction(_ sender: Any) {
        inputUpdate(Num: "3")
    }
    @IBAction func Input4ButtonAction(_ sender: Any) {
        inputUpdate(Num: "4")
    }
    @IBAction func Input5ButtonAction(_ sender: Any) {
        inputUpdate(Num: "5")
    }
    @IBAction func Input6ButtonAction(_ sender: Any) {
        inputUpdate(Num: "6")
    }
    @IBAction func Input7ButtonAction(_ sender: Any) {
        inputUpdate(Num: "7")
    }
    @IBAction func Input8ButtonAction(_ sender: Any) {
        inputUpdate(Num: "8")
    }
    @IBAction func Input9ButtonAction(_ sender: Any) {
        inputUpdate(Num: "9")
    }
    @IBAction func ClearButtonAction(_ sender: Any) {
        deleteUpdate()
    }
    
    
    func deleteUpdate() {
        Input1Label.text = ""
        Input2Label.text = ""
        Input3Label.text = ""
        Input4Label.text = ""
    }
    
    func inputUpdate(Num: String) {
        
        var count:Int = 1
        if (Input1Label.text != ""){
            count += 1
            if(Input2Label.text != ""){
                count += 1
                if(Input3Label.text != ""){
                    count += 1
                }
            }
        }
        
        switch count {
        case 1:
            Input1Label.text = Num
        case 2:
            Input2Label.text = Num
        case 3:
            Input3Label.text = Num
        case 4:
            if(Input4Label.text == ""){
                Input4Label.text = Num
            }
        default:
            Input1Label.text = Num
        }
    }
    
    @IBAction func InputButtonAction(_ sender: Any) {
        if(Input1Label.text != "" && Input2Label.text != "" && Input3Label.text != "" && Input4Label.text != ""){
            var b_Num:String = ""
            b_Num = Input1Label.text! + Input2Label.text! + Input3Label.text! + Input4Label.text!
            
            getInfo(b_Num: b_Num)
        }
    }
    
    func getInfo(b_Num:String){
        //let ipAddress = IPAddress.shared
        let ipAddress:String = "http://" + IPAddress
        let phpFileName = "getConfInfo.php"
        let b_Num = "b_Num=\(b_Num)"
        
        //var request = URLRequest(url: URL(string: "\(ipAddress.path)/\(phpFileName)?\(b_Num)")!)
        var request = URLRequest(url: URL(string: "\(ipAddress)/\(phpFileName)?\(b_Num)")!)
        request.httpMethod = "POST"
        
        let task = URLSession.shared.dataTask(with: request, completionHandler:{
            (data, response, error) in
            
            if error != nil {
                return
            }
            
            do{
                let decoder = JSONDecoder()
                let json = try decoder.decode(bookInfoJson.self, from: data!)
                
                self.b_Info = json
                print(json)
                
                
                
            }catch{
                self.cautionFlag = true
                print(error)
            }
        })
        task.resume()
        
        if(cautionFlag! == true){
            displayCaution()
        }
        
        if(b_Info != nil){
            let storyboard: UIStoryboard = self.storyboard!
            let nextView = storyboard.instantiateViewController(withIdentifier: "Confirm") as! ConfirmViewController
            nextView.b_Info = self.b_Info
            self.present(nextView, animated: true, completion: nil)
        }
    }
    
    func displayCaution(){
        let storyboard: UIStoryboard = self.storyboard!
        let nextView = storyboard.instantiateViewController(withIdentifier: "Caution") as! CautionViewController
        self.present(nextView, animated: true, completion: nil)
    }
    
    @IBAction func configButtonAction(_ sender: Any) {
        let storyboard: UIStoryboard = self.storyboard!
        let nextView = storyboard.instantiateViewController(withIdentifier: "Config") as! ConfigViewController
        self.present(nextView, animated: true, completion: nil)
        
    }
    
    
    
    
    /*
    // MARK: - Navigation

    // In a storyboard-based application, you will often want to do a little preparation before navigation
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        // Get the new view controller using segue.destinationViewController.
        // Pass the selected object to the new view controller.
    }
    */

}
