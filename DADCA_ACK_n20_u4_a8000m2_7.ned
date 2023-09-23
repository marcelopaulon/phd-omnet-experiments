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

network DADCA_ACK_n20_u4_a8000m2_7
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
        @display("i=misc/sensor;p=428.87,7942.03");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1109.24,7779.67");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1970.39,7809.94");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2332.11,7764.68");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3464.84,7803.94");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=74.16,7664.85");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=628.02,7636.83");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1547.19,7716.16");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2689.77,7701.90");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3051.89,7739.22");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3818.49,7747.75");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4367.41,7619.92");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4762.63,7705.35");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5161.99,7527.11");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5745.11,7598.20");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=335.72,7273.47");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=847.96,7300.70");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1246.42,7285.44");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2262.06,7328.23");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3315.70,7454.87");
    }

}

