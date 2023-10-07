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

network DADCA_ACK_n20_u3_a8000m2_0
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
        @display("i=misc/sensor;p=1815.49,7831.92");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2359.51,7896.96");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2781.81,7983.08");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3582.56,7985.30");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4593.85,7908.14");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5206.01,7796.03");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=336.08,7711.59");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=935.36,7727.97");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1458.05,7742.81");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2336.32,7539.92");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4109.56,7728.69");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4804.72,7516.08");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5923.17,7576.93");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=693.10,7327.24");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1216.97,7484.81");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1999.25,7385.83");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3029.08,7488.27");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3646.26,7397.01");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4260.68,7349.05");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5481.13,7473.76");
    }

}

