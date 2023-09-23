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

network DADCA_ACK_n50_u4_a8000m2_6
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
        @display("i=misc/sensor;p=497.94,7918.95");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1195.91,7930.95");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1676.83,7843.42");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3212.30,7914.71");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3812.93,7811.05");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4289.47,7961.55");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4898.13,7873.83");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5757.78,7956.21");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=124.62,7517.05");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=753.26,7629.59");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1431.56,7510.72");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2662.84,7677.63");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3132.23,7555.60");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4732.52,7552.80");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=515.34,7357.81");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1804.24,7262.29");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2139.49,7448.82");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3580.56,7398.00");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4083.61,7430.93");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5256.72,7444.24");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5828.88,7284.49");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1189.58,7177.75");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2288.97,7050.46");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2807.84,7241.43");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3122.50,7012.01");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3642.13,7030.35");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4696.87,7048.01");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5183.09,7034.45");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=910.55,6811.60");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1673.24,6911.59");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4213.06,6967.70");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=458.30,6733.80");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1374.02,6718.13");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2232.26,6677.96");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2912.60,6671.65");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3591.53,6632.43");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4545.61,6710.88");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4969.00,6527.30");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5531.50,6754.16");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=82.80,6377.52");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=452.47,6282.75");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1871.59,6453.84");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2641.02,6345.92");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4002.75,6487.39");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5402.29,6311.02");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=824.03,6124.85");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1477.98,6106.31");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2008.61,6087.28");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2383.33,6100.01");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3230.02,6069.39");
    }

}

