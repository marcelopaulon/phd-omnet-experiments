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

network DADCA_ACK_n50_u3_a8000m2_2
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
        @display("i=misc/sensor;p=1115.03,7903.58");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2732.13,7952.60");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3310.19,7838.52");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3941.31,7839.50");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5125.32,7764.85");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5631.68,7875.83");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1642.52,7687.26");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2214.18,7571.38");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2760.72,7589.60");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5975.83,7610.85");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1050.57,7486.88");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3001.84,7295.98");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3721.43,7314.02");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4276.24,7375.23");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4826.52,7416.39");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5474.07,7384.43");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=17.68,7042.78");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1112.99,7061.48");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1476.33,7169.70");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1821.87,7061.84");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2412.27,7123.35");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5784.34,7084.39");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=640.77,6930.00");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3324.72,6981.91");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4228.88,6911.64");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4794.77,6981.94");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5255.48,6920.58");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=393.80,6588.37");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1304.62,6748.38");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1923.26,6711.89");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2776.77,6653.48");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3322.64,6535.20");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3687.04,6703.96");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4650.41,6515.62");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5006.86,6555.13");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5501.31,6636.50");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=910.08,6471.60");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1718.00,6391.60");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2342.63,6384.37");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4209.26,6486.17");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=200.74,6022.28");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=529.33,6175.16");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1366.38,6178.59");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3041.96,6066.90");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3823.50,6106.65");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4499.19,6120.78");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4919.20,6041.19");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5451.61,6237.24");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5850.19,6110.91");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1041.50,5981.59");
    }

}

