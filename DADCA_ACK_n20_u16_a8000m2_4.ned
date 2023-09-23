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

network DADCA_ACK_n20_u16_a8000m2_4
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
        @display("i=misc/sensor;p=2797.57,7815.61");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3272.22,7930.70");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3860.92,7784.37");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4639.20,7896.61");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5550.24,7871.81");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=243.13,7535.69");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1124.72,7641.63");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1804.93,7691.71");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3293.95,7547.79");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4398.56,7566.40");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5207.96,7720.90");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5939.76,7657.26");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=691.67,7474.33");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2171.57,7459.54");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2531.21,7277.65");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2934.35,7474.29");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5496.67,7490.87");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5839.16,7301.44");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1246.69,7136.67");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3111.60,7036.75");
    }

}

