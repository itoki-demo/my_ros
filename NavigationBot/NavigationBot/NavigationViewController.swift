//
//  NavigationViewController.swift
//  NavigationBot
//
//  Created by 浜田 on 2018/09/04.
//  Copyright © 2018 swiftbigg. All rights reserved.
//

import UIKit

class NavigationViewController: UIViewController {

    var b_Info:ReceptionViewController.bookInfoJson?
    var getInfoTimer:Timer?
    var animationTimer:Timer?
    var botStatus:String?
    
    
    @IBOutlet weak var penguinMove: UIButton!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        self.getInfoTimer = Timer.scheduledTimer(timeInterval: 1, target: self, selector: #selector(NavigationViewController.getInfoUpdate), userInfo: nil, repeats: true)
        
        self.animationTimer = Timer.scheduledTimer(timeInterval: 0.5, target: self, selector: #selector(NavigationViewController.updateAnimation), userInfo: nil, repeats: true)
        // Do any additional setup after loading the view.
    }

    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }
    
    @objc func updateAnimation(){
        UIView.animate(withDuration: 0.5, delay: 0.0, options: [.curveEaseIn, .autoreverse], animations: {
            self.penguinMove.center.y += 100.0
        }) { _ in
            self.penguinMove.center.y -= 100.0
        }
    }
    
    @objc func getInfoUpdate(){
        getStatus()
        switch self.botStatus {
        case "navigation":
            print()
            
            
        case "door":
            let storyboard: UIStoryboard = self.storyboard!
                let nextView = storyboard.instantiateViewController(withIdentifier: "Arrive") as! ArriveViewController
                nextView.b_Info = self.b_Info!
                self.present(nextView, animated: true, completion: nil)
                
            
            
            
        case "room":
            let storyboard: UIStoryboard = self.storyboard!
            let nextView = storyboard.instantiateViewController(withIdentifier: "Conference") as! ConferenceViewController
            nextView.b_Info = self.b_Info!
            self.present(nextView, animated: true, completion: nil)
            
        case "reception":
            let storyboard: UIStoryboard = self.storyboard!
            let nextView = storyboard.instantiateViewController(withIdentifier: "Reception") as! ReceptionViewController
            self.present(nextView, animated: true, completion: nil)
        default:
            print()
        }
    }
    
    func getStatus(){
        var s:String?
        //let ipAddress = IPAddress.shared
        let ipAddress:String = "http://" + IPAddress
        let phpFileName = "get_status.php"
        
        //var request = URLRequest(url: URL(string: "\(ipAddress.path)/\(phpFileName)")!)
        var request = URLRequest(url: URL(string: "\(ipAddress)/\(phpFileName)")!)
        request.httpMethod = "POST"
        
        let task = URLSession.shared.dataTask(with: request, completionHandler:{
            (data, response, error) in
            
            if error != nil {
                return
            }
            do{
                s = String(data: data!, encoding: .utf8)
                var splitStatus:[String]
                splitStatus = s!.components(separatedBy: "_")
                if(splitStatus.count > 1){
                    s! = splitStatus[2]
                }else{
                    s! = splitStatus[0]
                }
                self.botStatus = s!
                print("data:\(s!)")
                
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
