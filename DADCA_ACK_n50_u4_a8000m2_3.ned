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

network DADCA_ACK_n50_u4_a8000m2_3
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
        @display("i=misc/sensor;p=309.53,7883.63");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=753.45,7923.03");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1078.45,7773.07");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1653.47,7772.47");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2084.29,7976.43");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2446.45,7998.96");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3337.32,7889.91");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4940.67,7893.96");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5327.19,7965.84");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5787.21,7980.55");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2011.81,7581.16");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2784.98,7505.11");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3881.28,7738.96");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4298.69,7634.30");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4904.29,7520.20");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5261.64,7551.80");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5599.03,7668.22");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=319.49,7458.30");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=790.13,7424.49");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1462.17,7320.42");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2316.96,7365.24");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3179.44,7289.65");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3681.72,7429.30");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5916.08,7432.86");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1003.51,7107.79");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4178.91,7128.68");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5210.37,7117.60");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5726.23,7064.85");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=443.83,6950.31");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1889.92,6950.80");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2448.07,6997.66");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2944.09,6993.13");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3306.34,6816.65");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4388.43,6786.81");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4773.01,6994.04");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5494.00,6788.91");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=868.35,6645.38");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1511.44,6744.73");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2218.67,6707.97");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3694.86,6755.07");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=95.41,6487.52");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1690.41,6404.62");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2386.00,6396.23");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2847.03,6298.84");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3566.54,6415.61");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4040.85,6268.12");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4680.85,6437.15");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5223.42,6398.43");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5813.23,6453.45");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=347.38,6235.60");
    }

}

