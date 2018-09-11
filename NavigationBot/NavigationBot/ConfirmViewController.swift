//
//  ConfirmViewController.swift
//  NavigationBot
//
//  Created by 浜田 on 2018/09/04.
//  Copyright © 2018 swiftbigg. All rights reserved.
//

import UIKit

class ConfirmViewController: UIViewController {

    var b_Info:ReceptionViewController.bookInfoJson?
    
    
    @IBOutlet weak var dateLabel: UILabel!
    @IBOutlet weak var roomLabel: UILabel!
    @IBOutlet weak var subscriberLabel: UILabel!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        dateLabel.text = b_Info?.b_Date
        roomLabel.text = b_Info?.b_RoomName
        subscriberLabel.text = b_Info?.b_Name

        // Do any additional setup after loading the view.
    }

    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }
    
    @IBAction func okButtonAction(_ sender: Any) {
        //let ipAddress = IPAddress.shared
        let ipAddress:String = "http://" + IPAddress
        print("ip=" + IPAddress)
        let phpFileName = "update_roomname.php"
        print("RoomName = " + (b_Info?.b_RoomName)!)
        let roomname = "room_name=" + (b_Info?.b_RoomName)!
        
        //var request = URLRequest(url: URL(string: "\(ipAddress.path)/\(phpFileName)?\(roomname)")!)
        var request = URLRequest(url: URL(string: "\(ipAddress)/\(phpFileName)?\(roomname)")!)
        request.httpMethod = "POST"
        
        let task = URLSession.shared.dataTask(with: request, completionHandler:{
            (data, response, error) in
            
            if error != nil {
                return
            }
            
            do{
                
            }catch{
                print(error)
            }
        })
        task.resume()
        
        updateStatus(status: "navigation")
        
        
        let storyboard: UIStoryboard = self.storyboard!
        let nextView = storyboard.instantiateViewController(withIdentifier: "Navigation") as! NavigationViewController
        nextView.b_Info = self.b_Info
        self.present(nextView, animated: true, completion: nil)
        
        
    }
    
    @IBAction func cancelButtonAction(_ sender: Any) {
        let storyboard: UIStoryboard = self.storyboard!
        let nextView = storyboard.instantiateViewController(withIdentifier: "Reception") as! ReceptionViewController
        self.present(nextView, animated: true, completion: nil)
    }
    
    func updateStatus(status:String){
        //let ipAddress = IPAddress.shared
        let ipAddress = "http://" + IPAddress
        let phpFileName = "update_status.php"
        let status = "status=\(status)"
        //var request = URLRequest(url: URL(string: "\(ipAddress.path)/\(phpFileName)?\(status)")!)
        var request = URLRequest(url: URL(string: "\(ipAddress)/\(phpFileName)?\(status)")!)
        request.httpMethod = "POST"
        let task = URLSession.shared.dataTask(with: request, completionHandler:{
            (data, response, error) in
            if error != nil {
                return
            }
            do{
            }
        })
        task.resume()
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
