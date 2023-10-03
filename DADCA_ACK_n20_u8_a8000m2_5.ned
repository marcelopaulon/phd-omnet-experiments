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

network DADCA_ACK_n20_u8_a8000m2_5
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
        @display("i=misc/sensor;p=1324.57,7957.16");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3327.46,7804.87");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4315.76,7965.71");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4894.22,7893.17");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5507.55,7758.63");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5922.16,7876.17");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=851.23,7735.99");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1614.33,7577.44");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1950.28,7747.85");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2354.49,7507.24");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2934.09,7736.37");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5184.19,7598.33");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5985.17,7507.65");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=634.21,7325.83");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1147.41,7316.44");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3245.89,7413.03");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4079.43,7467.88");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4425.99,7379.57");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5541.75,7324.77");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1736.43,7089.59");
    }

}

