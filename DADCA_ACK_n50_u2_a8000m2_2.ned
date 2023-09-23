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

network DADCA_ACK_n50_u2_a8000m2_2
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
        @display("i=misc/sensor;p=646.92,7973.68");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1092.41,7883.09");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1448.12,7986.36");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2272.16,7867.73");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3549.89,7891.55");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4154.48,7959.26");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4596.09,7811.10");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=195.47,7617.89");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1680.30,7644.09");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2565.89,7635.43");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3232.35,7673.31");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3731.26,7545.78");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5727.72,7574.88");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2255.57,7323.21");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2882.18,7438.99");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3294.44,7325.65");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4572.83,7413.48");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5033.53,7397.21");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=13.22,7228.96");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=667.65,7203.78");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1454.84,7241.85");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1862.86,7072.05");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2896.08,7077.16");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3928.66,7232.28");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5732.88,7112.44");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=22.49,6846.78");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=803.87,6785.51");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1188.48,6939.22");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3443.46,6838.71");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4386.48,6922.68");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4924.34,6916.34");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5404.23,6961.31");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=466.55,6664.83");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1499.68,6680.88");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2800.69,6638.70");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3742.67,6583.13");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5311.99,6538.65");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1044.73,6498.24");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1366.47,6287.97");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1880.47,6508.03");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2242.00,6487.80");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3097.36,6309.87");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4246.25,6429.80");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4588.72,6337.88");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5682.93,6465.62");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1009.19,6088.96");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2166.00,6021.36");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2854.70,6020.49");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3415.75,6051.34");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3874.56,6084.66");
    }

}

