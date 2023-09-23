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

network DADCA_ACK_n20_u3_a8000m2_0
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
        @display("i=misc/sensor;p=2837.21,7802.73");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3186.68,7993.72");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3978.38,7902.44");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4838.77,7755.81");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5269.72,7833.41");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=354.84,7593.86");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=836.08,7590.40");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3484.27,7681.09");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4467.56,7732.08");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2416.37,7431.99");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3196.99,7396.69");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3585.37,7328.73");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4455.77,7372.07");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4834.16,7282.23");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5473.97,7417.43");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1172.82,7033.72");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1608.76,7025.96");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2874.53,7177.27");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4154.64,7124.39");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5100.76,7051.30");
    }

}

