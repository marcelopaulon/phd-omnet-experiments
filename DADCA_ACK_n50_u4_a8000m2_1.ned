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

network DADCA_ACK_n50_u4_a8000m2_1
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
        @display("i=misc/sensor;p=335.46,7997.71");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1693.14,7777.26");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2907.08,7786.94");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3844.16,7787.61");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4989.70,7970.49");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=332.95,7630.45");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=756.98,7696.57");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1327.20,7571.98");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2425.19,7524.34");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3284.21,7671.55");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3604.46,7507.49");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4331.23,7747.22");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5398.13,7751.69");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5962.76,7673.73");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2980.10,7355.36");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4483.87,7403.50");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5090.37,7299.08");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5469.98,7364.27");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=641.00,7054.64");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1124.09,7209.88");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1602.66,7217.39");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2082.34,7033.52");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2447.21,7018.25");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3381.29,7135.58");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3911.93,7226.31");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=174.49,6857.14");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1603.70,6853.05");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2835.82,6976.23");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4626.72,6775.68");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5204.99,6893.68");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=738.94,6524.79");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1204.28,6642.09");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1758.15,6530.57");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3327.76,6677.81");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4040.87,6687.25");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5712.32,6761.46");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=52.02,6421.88");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=397.14,6281.86");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1414.95,6272.35");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2118.04,6308.03");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3540.83,6334.56");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5075.59,6345.55");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5524.69,6273.71");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=812.97,6155.92");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2750.65,6175.99");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4038.58,6195.71");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4721.31,6120.09");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5968.00,6125.82");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=70.94,5886.72");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1162.36,5992.35");
    }

}

