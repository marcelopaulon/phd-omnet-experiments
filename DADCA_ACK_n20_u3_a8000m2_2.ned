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

network DADCA_ACK_n20_u3_a8000m2_2
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
        @display("i=misc/sensor;p=1987.18,7819.26");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2755.69,7944.12");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3502.92,7972.90");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4133.38,7886.74");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1394.13,7740.30");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2417.31,7649.67");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2936.57,7624.05");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3350.82,7588.39");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4427.86,7682.20");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4860.48,7573.75");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5406.87,7659.15");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5791.77,7650.77");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=135.08,7332.88");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=576.84,7481.71");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=909.18,7356.12");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1470.13,7398.19");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1858.93,7344.42");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3693.27,7414.03");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4325.50,7257.96");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5602.71,7267.30");
    }

}

