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

network DADCA_ACK_n50_u4_a8000m2_0
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
        @display("i=misc/sensor;p=181.17,7923.32");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=615.30,7959.11");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1662.41,7904.06");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2257.05,7966.16");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2794.98,7799.92");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3482.08,7777.95");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4072.79,7799.45");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=567.22,7603.38");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=992.31,7671.30");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1345.83,7725.93");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4635.23,7678.35");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5131.99,7698.52");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=234.87,7323.76");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=742.29,7260.03");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1360.18,7335.25");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1720.80,7343.41");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2173.67,7468.04");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2582.27,7410.48");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2935.20,7402.53");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3424.20,7360.81");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5113.14,7330.11");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5676.36,7465.05");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2178.43,7032.23");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3960.18,7136.12");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4528.30,7152.56");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5399.39,7018.46");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=216.85,6830.04");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1618.30,6894.69");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2794.20,6779.27");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3189.48,6967.51");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5106.18,6827.32");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=768.42,6638.75");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1289.22,6641.26");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3768.86,6576.47");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4238.27,6611.59");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5966.40,6632.35");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1857.29,6487.07");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2354.80,6352.13");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3428.90,6338.02");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5625.01,6406.41");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=113.00,6226.91");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=744.14,6091.17");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1905.54,6120.26");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3025.30,6154.09");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4267.67,6130.94");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5119.56,6169.52");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=204.62,5797.53");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1152.06,5950.10");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1510.50,6014.31");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2144.15,5848.95");
    }

}

