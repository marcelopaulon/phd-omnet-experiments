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

network DADCA_ACK_n50_u16_a8000m2_1
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
        @display("i=misc/sensor;p=2176.52,7900.90");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2590.36,7922.92");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3542.36,7844.71");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4697.16,7938.68");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=545.15,7741.91");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=994.91,7657.59");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1490.57,7712.55");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1886.66,7668.19");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2887.88,7670.01");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4019.92,7685.53");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4424.73,7580.74");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4800.29,7545.37");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5254.48,7600.36");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=250.10,7287.71");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1286.98,7369.46");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1637.54,7307.85");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3902.96,7273.30");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=734.54,7165.07");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1233.80,7020.24");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2585.28,7233.16");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3412.49,7246.08");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4310.98,7021.95");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4964.20,7159.40");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=453.39,6850.14");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1606.79,6844.80");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2079.35,6918.29");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5314.00,6925.55");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5939.79,6928.45");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=175.12,6608.37");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=858.38,6705.86");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2343.75,6639.07");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3037.35,6614.16");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3606.33,6622.80");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4023.21,6747.01");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4542.82,6744.42");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=582.35,6476.52");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1927.32,6439.79");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2502.71,6322.53");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3361.46,6274.24");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3894.71,6394.05");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4332.80,6397.32");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4825.41,6318.35");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5468.37,6468.98");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=548.37,6083.17");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=908.99,6126.53");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1519.71,6266.81");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4104.59,6109.20");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4548.47,6066.42");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5188.20,6103.65");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5798.22,6243.53");
    }

}

