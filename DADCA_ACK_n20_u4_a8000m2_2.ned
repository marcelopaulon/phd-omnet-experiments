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

network DADCA_ACK_n20_u4_a8000m2_2
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
        @display("i=misc/sensor;p=416.95,7900.20");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1052.65,7790.85");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2488.26,7821.21");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4339.51,7944.72");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5541.00,7907.16");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1962.27,7641.25");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3009.70,7682.48");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3427.20,7740.37");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3855.74,7539.31");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4482.95,7510.01");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5185.00,7712.56");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=263.15,7481.33");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=698.19,7483.05");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1219.73,7388.23");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1574.09,7436.04");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4847.37,7488.77");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5841.42,7477.82");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=24.43,7061.93");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=409.04,7023.93");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=987.00,7073.74");
    }

}

