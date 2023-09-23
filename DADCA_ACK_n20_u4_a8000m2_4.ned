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

network DADCA_ACK_n20_u4_a8000m2_4
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
        @display("i=misc/sensor;p=949.47,7922.13");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1590.93,7992.19");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2857.18,7975.93");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3512.62,7770.88");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4008.52,7907.12");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4662.41,7777.29");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5194.64,7832.71");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=345.10,7532.94");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=838.57,7569.11");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1410.85,7625.25");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2313.52,7603.18");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5686.24,7717.55");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1989.80,7437.81");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2706.61,7415.01");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3228.91,7257.68");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3785.39,7359.39");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4321.06,7403.64");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4755.78,7258.07");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5278.38,7427.16");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=32.70,7127.83");
    }

}

