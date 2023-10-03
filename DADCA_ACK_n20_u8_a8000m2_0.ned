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
        @display("i=misc/sensor;p=1499.46,7906.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2552.97,7833.17");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=5073.93,7960.42");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=679.77,7719.75");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1362.79,7536.78");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2037.98,7623.74");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4285.09,7741.83");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4734.42,7640.34");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5599.52,7526.85");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=402.82,7473.23");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=832.94,7342.63");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3310.14,7280.38");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3805.83,7468.37");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4463.62,7259.51");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5885.94,7295.39");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=292.48,7077.13");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1775.54,7146.07");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2179.01,7134.47");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2703.47,7144.60");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4956.91,7211.33");
    }

}

