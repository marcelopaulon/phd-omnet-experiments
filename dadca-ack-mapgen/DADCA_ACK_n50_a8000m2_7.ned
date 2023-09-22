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

network DADCA_ACK_n50_a8000m2_7
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
        @display("i=misc/sensor;p=70.61,7968.56");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=152.05,7973.04");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=244.70,7977.84");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=404.65,7992.25");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=713.86,7971.74");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=784.06,7957.11");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=906.35,7993.55");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=999.63,7985.33");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=341.88,7952.37");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=496.06,7924.95");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=609.97,7939.48");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=942.16,7939.11");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=128.31,7891.88");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=234.22,7904.32");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=365.32,7893.06");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=432.04,7908.81");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=533.22,7871.90");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=733.43,7904.77");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=858.39,7884.28");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=981.80,7888.17");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=187.61,7841.24");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=280.45,7857.64");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=605.56,7856.50");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=902.43,7829.89");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=109.28,7786.97");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=298.93,7787.90");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=384.22,7787.17");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=493.59,7824.54");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=568.84,7800.69");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=685.94,7810.63");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=803.83,7809.70");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=214.56,7773.11");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=864.68,7770.63");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=33.93,7730.65");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=136.08,7712.16");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=241.33,7711.07");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=315.35,7698.74");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=491.02,7721.83");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=630.11,7724.45");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=732.80,7717.61");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=799.78,7724.52");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=969.57,7725.16");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=382.86,7685.97");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=556.85,7691.22");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=843.30,7664.40");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=913.90,7686.30");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=80.98,7630.24");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=470.38,7650.49");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=614.91,7654.46");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=698.98,7651.47");
    }

}

