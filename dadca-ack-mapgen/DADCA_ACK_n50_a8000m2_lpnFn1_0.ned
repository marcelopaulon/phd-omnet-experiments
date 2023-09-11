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

network DADCA_ACK_n50_a8000m2_lpnFn1_0
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
        @display("i=misc/sensor,p=26.76,148.94");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=22.78,81.60");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=47.56,18.53");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=134.63,33.33");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=226.86,9.49");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=139.50,170.99");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=152.09,232.86");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=142.18,292.42");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=64.40,311.75");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=231.11,285.87");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=222.47,356.99");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=298.12,364.68");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=365.68,357.60");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=293.65,290.75");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=57.78,399.82");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=40.38,466.05");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=119.24,434.32");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=454.33,390.65");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=396.43,429.17");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=326.41,423.80");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=275.99,505.06");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=256.38,585.39");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=190.57,587.87");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=131.40,570.09");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=65.67,527.97");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=43.33,600.34");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=25.14,662.16");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=119.53,680.90");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=177.39,724.94");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=214.21,813.05");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=231.98,905.75");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=298.06,847.88");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=315.91,754.75");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=360.52,689.12");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=444.25,696.29");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=435.99,756.09");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=295.15,640.07");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=194.51,502.94");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=259.41,435.87");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=153.35,358.60");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=80.20,238.00");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=195.54,94.59");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=276.45,108.90");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=301.20,20.83");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=375.70,23.11");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=219.65,203.94");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=333.57,154.60");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=7.35,252.34");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=292.07,204.60");
    }

}

