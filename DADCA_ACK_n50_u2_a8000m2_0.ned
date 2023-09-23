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

network DADCA_ACK_n50_u2_a8000m2_0
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
        @display("i=misc/sensor;p=619.44,7842.07");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1229.19,7841.80");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1653.14,7821.62");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2025.62,7811.28");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2388.42,7967.02");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2881.53,7867.66");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4479.72,7884.47");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4941.84,7827.66");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5613.44,7988.65");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=74.89,7737.23");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2443.05,7540.36");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3516.71,7595.30");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4067.01,7624.14");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4608.03,7515.52");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5802.05,7604.82");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4.12,7350.13");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=798.68,7302.55");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1188.74,7493.58");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2991.35,7367.15");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5363.74,7380.45");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1445.52,7165.83");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2121.82,7153.05");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2577.06,7213.49");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4818.57,7227.85");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=327.72,6896.83");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3742.83,7002.38");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4568.94,6867.57");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5790.04,6921.84");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=970.07,6706.57");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1538.18,6746.91");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1953.71,6685.54");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2388.59,6679.83");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2801.82,6704.59");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3356.04,6755.57");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3638.31,6538.99");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3963.85,6679.09");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5016.42,6703.43");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5527.35,6661.75");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=316.84,6372.84");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=812.39,6343.03");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1778.03,6338.58");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2958.11,6302.88");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4423.11,6400.69");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5243.75,6339.53");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1391.49,6206.25");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2354.25,6245.71");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3795.08,6225.64");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4881.14,6211.56");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5960.49,6169.22");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=218.93,5931.37");
    }

}

