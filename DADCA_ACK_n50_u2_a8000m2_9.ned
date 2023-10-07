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

network DADCA_ACK_n50_u2_a8000m2_9
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
        @display("i=misc/sensor;p=76.77,7872.73");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1973.19,7881.81");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2478.09,7876.03");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2844.07,7968.50");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3239.71,7896.22");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4339.07,7872.61");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=769.43,7573.65");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1342.86,7659.39");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1794.50,7527.31");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3657.08,7733.83");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4552.19,7591.55");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5040.49,7599.85");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=434.90,7468.23");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2677.15,7488.64");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4151.20,7461.97");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=178.25,7100.54");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=991.50,7061.02");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1323.63,7218.51");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2044.34,7201.46");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2631.54,7041.91");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3210.23,7246.99");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3778.17,7185.96");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4424.20,7191.83");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4906.38,7052.98");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5267.47,7160.52");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5944.69,7247.38");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=654.69,6965.21");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1583.29,6884.44");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3056.43,6781.41");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3968.67,6872.09");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1175.62,6527.94");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1926.36,6549.74");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2421.34,6657.76");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3498.06,6615.15");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4482.36,6592.52");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5427.18,6710.89");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5870.56,6639.70");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=671.81,6465.20");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2467.89,6306.26");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3082.27,6330.44");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3913.65,6394.01");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5119.59,6500.43");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1095.23,6087.15");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1758.31,6042.43");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2787.70,6040.35");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4852.28,6096.38");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5592.40,6152.08");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2243.59,5990.46");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4238.94,5959.38");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5275.20,5858.14");
    }

}

