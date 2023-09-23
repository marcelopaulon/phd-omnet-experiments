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

network DADCA_ACK_n50_u2_a8000m2_5
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
        @display("i=misc/sensor;p=625.94,7790.84");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1739.98,7949.25");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2146.79,7966.84");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2760.99,7804.13");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4158.57,7910.98");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4590.93,7844.90");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4995.39,7858.36");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=200.90,7686.52");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=951.32,7538.39");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1995.42,7642.50");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3418.44,7692.39");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5622.84,7604.46");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=648.25,7295.98");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2453.55,7282.45");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3861.14,7436.01");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4843.48,7488.38");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5391.45,7278.47");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=149.06,7244.36");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1560.37,7213.18");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2873.41,7175.04");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3605.63,7094.47");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4182.74,7158.13");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=435.61,6893.43");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=796.30,6891.10");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1179.92,6963.76");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1841.74,7000.34");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3060.80,6769.51");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5140.58,6812.18");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5625.13,6768.42");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=360.48,6546.39");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=849.57,6527.99");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1521.44,6668.40");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2083.12,6660.07");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2699.14,6688.12");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3800.63,6518.00");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4398.42,6651.79");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1217.99,6378.94");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2405.08,6471.34");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3431.14,6512.60");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4218.57,6307.37");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4748.67,6460.30");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5227.92,6386.47");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5724.96,6346.52");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=119.19,6146.13");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=569.49,6203.96");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=936.16,6168.94");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1573.06,6230.34");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2149.59,6084.49");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3163.19,6262.10");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3657.74,6026.39");
    }

}

