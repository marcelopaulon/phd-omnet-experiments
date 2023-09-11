package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n50_a8000m2_lpnFn1_5
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=90.50,193.22");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=27.31,247.82");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=150.80,225.58");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=108.51,269.32");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=166.92,346.33");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=262.43,339.64");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=301.42,390.81");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=363.33,315.18");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=207.74,278.15");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=254.87,228.34");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=327.33,248.81");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=393.22,195.41");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=423.05,109.47");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=419.91,47.92");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=346.27,24.60");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=342.55,153.34");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=254.01,143.62");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=171.76,140.00");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=137.56,41.23");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=29.69,69.42");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=1.73,343.28");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=79.16,381.47");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=33.45,462.75");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=202.58,414.79");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=162.09,466.78");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=141.22,524.20");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=10.32,577.88");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=106.53,602.41");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=93.77,661.05");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=217.71,611.08");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=251.22,487.77");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=231.46,548.61");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=42.39,629.70");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=9.91,709.00");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=0.96,792.85");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=59.03,871.04");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=132.60,859.33");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=104.57,917.58");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=119.68,987.15");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=47.65,1030.19");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=35.23,945.70");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=173.61,909.58");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=23.52,1133.16");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=20.51,1218.47");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=96.65,1177.39");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=96.58,1112.03");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=162.97,1067.89");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=233.28,1022.26");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=210.74,962.46");
    }

}

