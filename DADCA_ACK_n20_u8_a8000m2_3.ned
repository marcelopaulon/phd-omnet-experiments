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

network DADCA_ACK_n20_u8_a8000m2_3
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
        @display("i=misc/sensor;p=781.64,7832.67");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1446.44,7780.19");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2686.43,7991.15");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3449.09,7754.01");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4458.32,7832.40");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5116.68,7777.85");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=240.97,7656.40");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1931.34,7671.01");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2779.22,7615.10");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3779.48,7542.24");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5649.24,7588.97");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=402.32,7259.44");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=816.97,7463.58");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2481.07,7377.99");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3114.87,7369.67");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4511.13,7418.58");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4964.23,7438.96");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5973.93,7443.32");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=934.09,7110.18");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1408.76,7205.54");
    }

}

