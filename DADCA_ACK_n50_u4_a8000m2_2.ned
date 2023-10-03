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

network DADCA_ACK_n50_u4_a8000m2_2
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
        @display("i=misc/sensor;p=846.99,7836.21");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2224.17,7881.69");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2794.27,7943.63");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4494.70,7931.82");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5037.81,7896.88");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5843.42,7860.59");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1206.21,7542.02");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1529.54,7704.17");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1890.27,7656.42");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3461.79,7751.65");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4454.88,7547.38");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=196.03,7372.34");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=872.39,7334.47");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1522.63,7352.35");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2745.46,7300.05");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3074.95,7495.45");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3941.36,7401.66");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4790.47,7443.34");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5358.06,7476.50");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1114.21,7018.39");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1932.41,7187.59");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3048.07,7064.76");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3556.34,7253.55");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4479.94,7183.27");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5106.69,7065.32");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5768.57,7034.52");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=601.89,6974.87");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1563.85,6972.11");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2436.04,6946.17");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4009.85,6973.36");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4756.15,6902.63");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5376.13,6817.46");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=329.01,6657.34");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1638.21,6586.44");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2892.25,6554.15");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3495.63,6722.99");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4482.92,6528.74");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=91.79,6392.24");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=702.24,6309.54");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1157.37,6367.75");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2086.74,6395.12");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2477.34,6398.02");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3475.62,6300.21");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3909.23,6493.69");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4845.89,6457.48");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2054.45,6021.88");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2725.22,6038.75");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3128.90,6151.40");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4074.81,6183.58");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4512.98,6049.09");
    }

}

