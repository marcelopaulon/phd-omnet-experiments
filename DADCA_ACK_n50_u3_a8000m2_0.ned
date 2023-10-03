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

network DADCA_ACK_n50_u3_a8000m2_0
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
        @display("i=misc/sensor;p=470.45,7862.79");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1393.63,7827.88");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2674.75,7763.82");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3586.16,7885.59");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4497.31,7939.37");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5332.07,7889.58");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=612.21,7534.59");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=954.45,7695.90");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2224.05,7542.08");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3149.05,7539.68");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5111.10,7616.84");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5802.42,7540.85");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=79.37,7467.90");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1246.42,7315.03");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1792.27,7416.23");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2612.36,7418.16");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3604.95,7480.27");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4087.21,7442.59");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4638.08,7327.82");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=750.56,7086.44");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1756.42,7043.42");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2674.96,7047.65");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3886.88,7111.75");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5329.20,7218.56");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=351.57,6930.73");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1050.09,6854.08");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1420.32,6920.49");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2265.89,6915.46");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3305.82,6812.31");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4185.56,6769.77");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4563.15,6964.47");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5036.33,6920.64");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5448.97,6775.42");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5780.27,6919.40");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2098.90,6518.11");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2459.43,6595.54");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2864.83,6743.84");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4748.21,6652.15");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=48.18,6459.14");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=580.32,6455.61");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1173.65,6388.21");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1683.15,6404.15");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2883.69,6387.63");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3359.18,6451.76");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3757.51,6424.76");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4477.15,6390.01");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5757.16,6375.75");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=286.09,6106.22");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2061.22,6027.67");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2449.83,6125.53");
    }

}

