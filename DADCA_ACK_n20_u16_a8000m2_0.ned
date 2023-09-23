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

network DADCA_ACK_n20_u16_a8000m2_0
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
        @display("i=misc/sensor;p=236.40,7934.49");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=561.83,7789.02");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1079.54,7929.39");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2315.98,7933.62");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3374.34,7936.09");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3779.21,7789.73");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5197.96,7941.32");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5843.96,7965.56");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1105.18,7523.33");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1631.75,7600.93");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2804.73,7664.16");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3176.68,7627.44");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4432.23,7575.46");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=26.71,7415.72");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=380.82,7429.73");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=769.28,7334.96");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1917.54,7332.84");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2361.29,7437.21");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3680.29,7430.04");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4080.92,7321.42");
    }

}

