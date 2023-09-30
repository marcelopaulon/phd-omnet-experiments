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

network DADCA_ACK_n50_u16_a8000m2_5
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
        @display("i=misc/sensor;p=724.61,7921.09");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2670.54,7860.88");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4276.80,7934.51");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=240.59,7631.05");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=864.52,7558.28");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1545.63,7544.61");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2080.61,7646.44");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2979.27,7637.02");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3471.22,7635.72");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3877.99,7747.20");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4275.20,7505.58");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4961.50,7677.46");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1997.60,7265.04");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2686.38,7379.09");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4632.17,7445.95");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5345.48,7430.28");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3.32,7233.80");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=435.73,7253.32");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1480.64,7091.75");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3526.11,7145.57");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5208.29,7085.82");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=240.71,6860.55");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1059.23,6802.11");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2387.67,6931.08");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2999.53,6980.46");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3423.19,6788.25");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4180.46,6957.26");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4544.46,6931.89");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=669.68,6701.02");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1895.05,6599.05");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4181.78,6544.08");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4775.15,6665.63");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5460.59,6538.72");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=114.88,6424.49");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=807.65,6355.00");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1280.56,6444.49");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3023.59,6341.00");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3811.85,6297.35");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4754.07,6277.64");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5074.89,6423.80");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5832.44,6424.34");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1547.93,6196.30");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2448.16,6204.83");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2817.08,6041.50");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3383.93,6211.03");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4232.19,6034.59");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=527.40,5874.78");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1311.83,5857.17");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1818.62,5941.24");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4897.60,5933.89");
    }

}

