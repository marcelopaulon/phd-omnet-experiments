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

network DADCA_ACK_n20_u8_a8000m2_1
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
        @display("i=misc/sensor;p=2206.44,7827.43");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3698.54,7816.70");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4168.51,7996.80");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5315.85,7894.92");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1194.98,7519.58");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2728.69,7649.44");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3106.20,7706.25");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3897.65,7528.67");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4257.36,7618.47");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4647.68,7709.85");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5616.63,7708.42");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1879.78,7326.78");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3023.45,7306.10");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5132.55,7477.08");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5765.62,7359.74");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=247.82,7207.17");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=709.62,7150.56");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1470.36,7159.63");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2360.94,7209.70");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3594.32,7142.65");
    }

}

