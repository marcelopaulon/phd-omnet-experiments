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

network DADCA_ACK_n50_u4_a8000m2_4
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
        @display("i=misc/sensor;p=205.65,7807.69");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=648.88,7957.15");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1137.41,7919.83");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2167.43,7920.63");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2547.35,7819.28");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3788.99,7794.36");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4166.85,7946.77");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=893.54,7602.78");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1622.27,7519.87");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3442.72,7675.85");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4448.23,7709.54");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4799.68,7717.32");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=415.16,7481.50");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1991.87,7364.31");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2608.01,7365.50");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3001.41,7448.13");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3681.59,7404.34");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4057.23,7430.51");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1335.70,7108.01");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1873.10,7015.51");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4463.97,7027.89");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4836.23,7248.55");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=756.94,6865.86");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1554.73,6821.41");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2531.47,6967.54");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3088.26,6789.32");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3367.81,7008.64");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3795.99,7008.25");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5048.06,6797.59");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=136.63,6636.81");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1346.70,6532.22");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4003.16,6628.15");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=461.09,6389.37");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1002.59,6449.43");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1705.03,6300.44");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2110.47,6447.83");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2739.56,6431.16");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3271.92,6484.90");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3766.17,6342.81");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4295.65,6332.34");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4733.45,6463.25");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5315.98,6412.30");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3156.48,6123.47");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4851.27,6089.54");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=221.83,5921.38");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=671.08,5913.60");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1042.48,5999.20");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1586.88,5929.88");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2056.51,5817.66");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2519.31,5967.79");
    }

}

