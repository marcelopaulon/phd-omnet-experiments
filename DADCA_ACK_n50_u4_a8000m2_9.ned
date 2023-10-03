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

network DADCA_ACK_n50_u4_a8000m2_9
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
        @display("i=misc/sensor;p=1513.73,7969.49");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2505.36,7946.06");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3144.92,7944.63");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3766.59,7845.93");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4152.79,7772.77");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5303.84,7969.12");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5833.12,7828.53");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=47.76,7685.08");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=695.98,7585.56");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1298.51,7693.22");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1985.79,7584.44");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4538.36,7646.32");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5046.85,7703.24");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5457.05,7619.38");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=250.86,7344.93");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1024.30,7357.50");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1679.50,7309.64");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2324.41,7482.94");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2824.57,7345.51");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3144.94,7497.85");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4838.92,7294.30");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5874.92,7326.47");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=647.83,7224.74");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1112.01,7016.54");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2034.12,7162.34");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2491.25,7125.95");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3482.54,7198.81");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4038.33,7256.23");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5468.95,7131.27");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3500.98,6806.52");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3854.11,6957.33");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4343.46,6819.45");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4995.80,6960.81");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=131.97,6749.32");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=566.82,6563.07");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1001.86,6674.96");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1383.81,6759.84");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1915.57,6689.73");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2500.85,6643.70");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2842.93,6731.70");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3164.85,6576.03");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3709.33,6522.18");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4824.85,6522.93");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5398.41,6664.18");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5836.25,6659.17");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1730.58,6380.51");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2702.30,6297.15");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4209.74,6278.61");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5489.80,6280.37");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=212.55,6104.78");
    }

}

