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
        @display("i=misc/sensor;p=1001.61,7904.75");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1442.37,7864.80");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2491.10,7999.03");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2881.60,7880.46");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4062.10,7976.04");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5266.30,7988.65");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2431.32,7597.04");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3469.67,7608.53");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4157.14,7611.03");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4639.81,7733.21");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5428.98,7556.00");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1806.21,7453.22");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3054.17,7323.03");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3784.62,7411.34");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4917.48,7324.88");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=803.47,7031.48");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2350.56,7070.24");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2720.38,7029.72");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4538.13,7070.10");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5679.93,7082.36");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=419.00,6973.03");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3410.11,6762.66");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3769.91,6949.18");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4902.68,6967.44");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=256.00,6554.00");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=605.98,6522.01");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1253.13,6671.95");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1813.64,6594.84");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2384.59,6537.70");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2901.81,6538.46");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4484.23,6585.69");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5185.00,6728.19");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4010.38,6445.46");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4884.73,6485.10");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5521.70,6354.24");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=43.78,6140.95");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=457.19,6089.10");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=820.00,6201.74");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1224.98,6154.02");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1571.51,6099.83");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2007.09,6211.02");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2580.34,6154.94");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4501.03,6033.99");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5012.32,6112.00");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2307.51,5927.34");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3231.76,6006.37");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3926.81,5851.96");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5586.43,5820.58");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5952.77,5915.80");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=342.11,5757.24");
    }

}

