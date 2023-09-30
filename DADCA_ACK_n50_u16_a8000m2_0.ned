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

network DADCA_ACK_n50_u16_a8000m2_0
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
        @display("i=misc/sensor;p=2336.21,7997.33");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2686.24,7911.85");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=5507.09,7854.18");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5907.71,7999.92");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=540.74,7690.94");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1170.00,7645.25");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1887.34,7733.59");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2573.46,7542.93");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3207.93,7685.45");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3794.74,7680.58");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4442.91,7583.82");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4950.09,7745.80");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5240.36,7548.60");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5828.70,7615.06");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=110.55,7296.83");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=578.22,7340.69");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3150.13,7284.72");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3966.58,7341.22");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4726.71,7292.53");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1144.62,7110.01");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1492.21,7175.94");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2082.59,7249.60");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3556.64,7141.67");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4151.26,7024.91");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5929.75,7041.84");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1807.94,6764.54");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2379.57,6824.28");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2841.94,6969.69");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3260.96,6892.23");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5651.74,6804.55");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=402.56,6685.53");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=750.77,6635.76");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1384.07,6625.12");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4029.58,6682.15");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5145.63,6536.05");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1893.01,6337.56");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2888.99,6329.30");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3554.22,6455.44");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4248.56,6331.51");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4589.61,6513.31");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5675.11,6322.88");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=510.47,6112.82");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1182.80,6245.91");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1606.07,6048.42");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2208.05,6059.17");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3210.76,6126.08");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3759.58,6100.76");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4472.55,6044.32");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4828.75,6101.65");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5436.67,6032.10");
    }

}

