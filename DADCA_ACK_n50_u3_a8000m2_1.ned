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

network DADCA_ACK_n50_u3_a8000m2_1
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
        @display("i=misc/sensor;p=447.96,7773.46");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1958.45,7796.26");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2988.50,7942.14");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3422.57,7779.78");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3903.43,7754.50");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4525.15,7951.62");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4877.43,7844.69");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1033.35,7510.45");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1393.19,7705.78");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2471.46,7521.37");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4616.92,7572.96");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1678.84,7493.27");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3004.82,7268.45");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4312.85,7376.81");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5060.48,7397.14");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5520.83,7503.69");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=123.66,7242.54");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1162.97,7070.77");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1598.74,7124.00");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2170.54,7230.80");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2562.41,7017.40");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3575.82,7079.67");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4009.71,7182.42");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4734.06,7121.16");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5275.24,7094.17");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5833.90,7117.58");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=585.11,6981.00");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3067.10,6790.16");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=252.71,6573.35");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=763.90,6541.40");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1263.96,6711.51");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1735.09,6574.50");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2116.84,6677.84");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4074.10,6666.59");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4773.00,6639.42");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5277.06,6519.79");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5777.42,6715.58");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1445.23,6310.31");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2711.66,6370.48");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3497.63,6446.53");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5758.62,6290.88");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=388.55,6125.92");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=942.85,6144.29");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1760.73,6157.85");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2121.17,6164.27");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2523.82,6022.09");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3151.08,6070.01");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3983.92,6166.17");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4536.21,6106.82");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4945.78,6061.36");
    }

}

