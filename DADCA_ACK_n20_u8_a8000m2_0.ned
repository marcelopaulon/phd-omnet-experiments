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

network DADCA_ACK_n20_u8_a8000m2_0
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
        @display("i=misc/sensor;p=122.57,7941.49");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1226.47,7886.41");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4079.33,7924.81");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4917.52,7842.78");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5831.77,7779.79");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=646.95,7649.37");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1961.40,7598.39");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2622.65,7744.35");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5294.45,7748.67");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=308.33,7422.31");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1037.73,7278.20");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1623.22,7271.32");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2993.33,7442.23");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3684.93,7493.79");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4066.62,7377.10");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4503.14,7445.85");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=537.52,7113.65");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2283.17,7108.42");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2652.11,7064.33");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4677.50,7140.43");
    }

}

