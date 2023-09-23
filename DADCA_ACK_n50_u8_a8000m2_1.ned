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

network DADCA_ACK_n50_u8_a8000m2_1
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
        @display("i=misc/sensor;p=305.26,7886.89");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1621.26,7963.17");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2598.97,7817.79");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2968.16,7984.42");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3684.77,7962.20");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1020.80,7634.50");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2356.29,7556.40");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3376.98,7568.71");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=422.01,7408.18");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1562.10,7387.64");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3894.79,7484.72");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=69.78,7157.23");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2062.46,7083.85");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2825.56,7210.89");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=832.69,6844.62");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1202.80,7001.22");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1527.49,6768.95");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3173.66,6950.11");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3653.61,6961.33");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=455.35,6721.12");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1193.82,6541.16");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2396.93,6669.19");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3031.00,6629.20");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1849.56,6337.22");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2619.17,6308.14");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3273.16,6294.54");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3684.07,6342.37");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=126.49,6027.97");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=545.12,6244.98");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=861.88,6060.66");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1463.11,6252.16");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2290.87,6102.94");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1177.90,5864.17");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2960.27,5823.33");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3380.40,5843.99");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3847.20,5908.80");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=246.18,5607.97");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=751.76,5581.48");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1557.86,5726.51");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2032.80,5726.32");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2384.38,5644.10");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2974.32,5383.75");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3344.55,5429.46");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3812.34,5369.80");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4188.09,5418.11");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=219.17,5082.53");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=573.29,5046.67");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1414.08,5269.96");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1858.44,5228.25");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2474.83,5209.67");
    }

}

